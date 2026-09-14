//===- StatepointMachinePasses.cpp - Simplify Statepoint related instrs ---===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// To be written.
//
//===----------------------------------------------------------------------===//
#include <atomic>
#include <cstdlib>

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveStacks.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/RegisterUsageInfo.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <list>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#define DEBUG_TYPE "statepoint-simplify"
using namespace llvm;
namespace {
struct DartSiblingStats {
  std::atomic<uint64_t> Count{0}, Uninit{0}, RegRecs{0};
  ~DartSiblingStats() {
    if (getenv("STATEPOINT_STATS") != nullptr)
      llvm::errs() << "SIBLING-LIVE slots_added=" << Count
                   << " dropped_uninit=" << Uninit << " reg_records=" << RegRecs
                   << "\n";
  }
};
} // namespace
static DartSiblingStats DartSiblingStatsInst;
#define DartSiblingLiveSlots DartSiblingStatsInst.Count
namespace {

class StatepointSimplify : public MachineFunctionPass {
public:
  StatepointSimplify();

  StringRef getPassName() const override { return "Statepoint Simplify"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  static char ID;

private:
  bool foldRelocateDef(MachineFunction &F);
  /// Remove the redundant move immediate from statepoints.
  bool removeVarFromStatepoints(MachineFunction &MF);

  bool removeVarFromStatepoint(MachineFunction &MF, MachineInstr *MI);

  void addStatepointObservedRecursive(Register r);

  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;
};

class StatepointRewrite : public MachineFunctionPass {
public:
  StatepointRewrite();

  StringRef getPassName() const override { return "Statepoint Rewrite"; }
  void getAnalysisUsage(AnalysisUsage &AU) const override;

  /// runOnMachineFunction - pass entry point
  bool runOnMachineFunction(MachineFunction &) override;
  static char ID;

private:
  bool rewriteStatepoints(MachineFunction &);
  bool rewriteStatepoint(MachineFunction &, MachineInstr *MI);

  /// True when a store to \p FI reaches \p SP on every path from the entry
  /// block. Uses the block-level must-reach result plus a scan of SP's own
  /// block.
  bool mustBeWrittenAt(int FI, const MachineInstr *SP) const;

  SmallVector<Register, 8> ObservedRegistersWithPhys;
  SmallVector<int, 8> ObservedStackSlots;
  // BUGFIX llvm23-statepoint-mustreach: per observed slot, the blocks where
  // every path has already stored to it, plus the blocks containing a store.
  DenseMap<int, BitVector> DefinitelyWritten;
  DenseMap<int, BitVector> BlockHasStore;
  // BUGFIX llvm23-slot-live-through-sibling: every virtual register of the
  // split family whose spill slot this is (VirtRegMap original), including the
  // siblings the allocator kept in registers.
  DenseMap<int, SmallVector<Register, 4>> SlotSiblingRegs;
  // BUGFIX llvm23-init-through-copies: value-aware "slot is initialised"
  // dataflow (see applyInit). InitIn[B] is the set of frame objects
  // initialised on entry to block B; VRegInit[R] is false for a register some
  // reload defined from an uninitialised slot; SlotHasStore is every frame
  // object with any store at all.
  bool slotInitAt(int FI, const MachineInstr *SP);
  void applyInit(const MachineInstr &MI, BitVector &S, bool *Flipped);
  void solveInit(MachineFunction &MF);
  bool fiInRange(int FI) const {
    return FI >= FIBase && FI < FIBase + static_cast<int>(NumFI);
  }
  unsigned fiIndex(int FI) const { return static_cast<unsigned>(FI - FIBase); }
  int FIBase = 0;
  unsigned NumFI = 0;
  const TargetInstrInfo *TII = nullptr;
  SmallVector<BitVector, 0> InitIn;
  DenseMap<Register, bool> VRegInit;
  DenseSet<int> SlotHasStore;
  SlotIndexes *Indexes;
  LiveIntervals *LIS;
  LiveStacks *LSS;
  VirtRegMap *VRM;
  MachineRegisterInfo *MRI;
};

static MachineInstr *getSingleDef(const MachineRegisterInfo &MRI,
                                  Register Reg) {
  if (!MRI.hasOneDef(Reg))
    return nullptr;
  auto defs_iterator = MRI.def_begin(Reg);
  return defs_iterator->getParent();
}
} // end of anonymous namespace

INITIALIZE_PASS(StatepointSimplify, DEBUG_TYPE, "Statepoint Simplify", false,
                false)

char StatepointSimplify::ID = 0;

StatepointSimplify::StatepointSimplify()
    : MachineFunctionPass(ID), MRI(nullptr), TII(nullptr) {
  PassRegistry &Registry = *PassRegistry::getPassRegistry();
  initializeStatepointSimplifyPass(Registry);
}

bool StatepointSimplify::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = foldRelocateDef(MF);
  Changed |= removeVarFromStatepoints(MF);
  return Changed;
}

bool StatepointSimplify::foldRelocateDef(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  const TargetSubtargetInfo &STI = MF.getSubtarget();
  TII = STI.getInstrInfo();
  bool Changed = false;

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      if (MI.getOpcode() == TargetOpcode::RELOCATE_DEF) {
        MI.setDesc(TII->get(TargetOpcode::COPY));
        Changed = true;
      }
    }
  }

  return Changed;
}

bool StatepointSimplify::removeVarFromStatepoints(MachineFunction &MF) {
  bool Changed = false;
  SmallVector<MachineInstr *, 8> WorkList;
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      switch (MI.getOpcode()) {
      case TargetOpcode::STATEPOINT:
        WorkList.emplace_back(&MI);
        break;
      }
    }
  }
  for (MachineInstr *MI : WorkList)
    Changed |= removeVarFromStatepoint(MF, MI);
  return Changed;
}

bool StatepointSimplify::removeVarFromStatepoint(MachineFunction &MF,
                                                 MachineInstr *StatePoint) {
  if (StatePoint->getOpcode() != TargetOpcode::STATEPOINT)
    llvm_unreachable("unexpected stackmap opcode");

  // BUGFIX llvm23-statepoint-sections: the meta sections of a STATEPOINT are

  StatepointOpers Op(StatePoint);
  const unsigned NumGCPtrIdx = Op.getNumGCPtrIdx();
  const unsigned AllocaMarkerIdx = Op.getNumAllocaIdx() - 1;
  const unsigned NumGcMapIdx = Op.getNumGcMapEntriesIdx();
  const int64_t NumGCPtrs = StatePoint->getOperand(NumGCPtrIdx).getImm();
  const int64_t NumGcMapEntries = StatePoint->getOperand(NumGcMapIdx).getImm();

  bool Changed = false;

  // The gc map holds base/derived pairs expressed as indices into the gc
  // pointer list. Once the pointers are gone the map cannot be kept
  // meaningful, and Dart's stackmap consumer does not use it, so drop it.
  if (NumGcMapEntries != 0) {
    for (unsigned I = NumGcMapIdx + 2 * NumGcMapEntries; I > NumGcMapIdx; --I)
      StatePoint->removeOperand(I);
    StatePoint->getOperand(NumGcMapIdx).setImm(0);
    Changed = true;
  }

  // GC pointer records sit between the count and the alloca section marker.
  int64_t Removed = 0;
  for (unsigned I = AllocaMarkerIdx; I-- > NumGCPtrIdx + 1;) {
    MachineOperand &MO = StatePoint->getOperand(I);
    if (!MO.isReg() || MO.getReg().isPhysical())
      continue;
    addStatepointObservedRecursive(MO.getReg());
    StatePoint->removeOperand(I);
    ++Removed;
    Changed = true;
  }
  if (Removed != 0)
    StatePoint->getOperand(NumGCPtrIdx).setImm(NumGCPtrs - Removed);

  return Changed;
}

void StatepointSimplify::addStatepointObservedRecursive(Register r) {
  MRI->addStatepointObserved(r);
  MachineInstr *DefMI = getSingleDef(*MRI, r);
  if (!DefMI)
    return;
  if (!DefMI->isFullCopy())
    return;
  Register SrcReg = DefMI->getOperand(1).getReg();
  if (MRI->isStatepointObserved(SrcReg))
    return;
  addStatepointObservedRecursive(SrcReg);
}

#undef DEBUG_TYPE
#define DEBUG_TYPE "statepoint-rewrite"
char StatepointRewrite::ID = 0;

INITIALIZE_PASS_BEGIN(StatepointRewrite, DEBUG_TYPE,
                      "Rewrite Statepoints PostRA", false, false)
INITIALIZE_PASS_DEPENDENCY(SlotIndexesWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LiveIntervalsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LiveStacksWrapperLegacy)
INITIALIZE_PASS_DEPENDENCY(VirtRegMapWrapperLegacy)
INITIALIZE_PASS_END(StatepointRewrite, DEBUG_TYPE, "Rewrite Statepoints PostRA",
                    false, false)

StatepointRewrite::StatepointRewrite()
    : MachineFunctionPass(ID), Indexes(nullptr), LSS(nullptr), VRM(nullptr) {
  PassRegistry &Registry = *PassRegistry::getPassRegistry();
  initializeStatepointRewritePass(Registry);
}

void StatepointRewrite::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<SlotIndexesWrapperPass>();
  AU.addRequired<LiveIntervalsWrapperPass>();
  AU.addRequired<LiveStacksWrapperLegacy>();
  AU.addRequired<VirtRegMapWrapperLegacy>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

bool StatepointRewrite::runOnMachineFunction(MachineFunction &MF) {
  Indexes = &getAnalysis<SlotIndexesWrapperPass>().getSI();
  LIS = &getAnalysis<LiveIntervalsWrapperPass>().getLIS();
  LSS = &getAnalysis<LiveStacksWrapperLegacy>().getLS();
  VRM = &getAnalysis<VirtRegMapWrapperLegacy>().getVRM();
  MRI = &MF.getRegInfo();
  return rewriteStatepoints(MF);
}

bool StatepointRewrite::mustBeWrittenAt(int FI, const MachineInstr *SP) const {
  auto DW = DefinitelyWritten.find(FI);
  if (DW == DefinitelyWritten.end())
    return false;
  const MachineBasicBlock *BB = SP->getParent();
  const unsigned B = BB->getNumber();
  if (DW->second.test(B))
    return true;
  auto BH = BlockHasStore.find(FI);
  if (BH == BlockHasStore.end() || !BH->second.test(B))
    return false;
  // A store inside this block only counts if it precedes the statepoint.
  for (const MachineInstr &MI : *BB) {
    if (&MI == SP)
      break;
    if (!MI.mayStore())
      continue;
    for (const MachineMemOperand *MMO : MI.memoperands()) {
      const PseudoSourceValue *PSV = MMO->getPseudoValue();
      if (PSV && PSV->kind() == PseudoSourceValue::FixedStack &&
          cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex() == FI)
        return true;
    }
  }
  return false;
}

void StatepointRewrite::applyInit(const MachineInstr &MI, BitVector &S,
                                  bool *Flipped) {
  int FI = 0;
  if (MI.mayLoad()) {
    if (Register R = TII->isLoadFromStackSlot(MI, FI)) {
      // A reload carries the initialisation state of its slot into the
      // register. A register with several defs is initialised only if every
      // reload that defines it read an initialised slot.
      if (R.isVirtual() && fiInRange(FI)) {
        auto It = VRegInit.try_emplace(R, true).first;
        if (It->second && !S.test(fiIndex(FI))) {
          It->second = false;
          if (Flipped)
            *Flipped = true;
        }
      }
    }
  }
  if (!MI.mayStore())
    return;
  if (Register Src = TII->isStoreToStackSlot(MI, FI)) {
    if (!fiInRange(FI))
      return;
    bool Init = true;
    if (Src.isVirtual()) {
      auto It = VRegInit.find(Src);
      if (It != VRegInit.end())
        Init = It->second;
    }
    if (Init)
      S.set(fiIndex(FI));
    else
      S.reset(fiIndex(FI));
    return;
  }
  // Any other store into a frame object: the stored value is not a plain
  // register we can trace, treat it as a definition (the old behaviour).
  for (const MachineMemOperand *MMO : MI.memoperands()) {
    const PseudoSourceValue *PSV = MMO->getPseudoValue();
    if (PSV && PSV->kind() == PseudoSourceValue::FixedStack) {
      const int F = cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex();
      if (fiInRange(F))
        S.set(fiIndex(F));
    }
  }
}

void StatepointRewrite::solveInit(MachineFunction &MF) {
  const unsigned NumBlocks = MF.getNumBlockIDs();
  VRegInit.clear();
  InitIn.assign(NumBlocks, BitVector(NumFI, true));
  SmallVector<BitVector, 0> Out(NumBlocks, BitVector(NumFI, true));
  ReversePostOrderTraversal<MachineFunction *> RPOT(&MF);
  // Must-dataflow (AND confluence): start from "everything initialised" and
  // only ever clear bits, i.e. iterate to the greatest fixed point. VRegInit
  // is monotone as well (true -> false only), so the loop terminates.
  bool Changed = true;
  while (Changed) {
    Changed = false;
    for (MachineBasicBlock *MBB : RPOT) {
      const unsigned B = MBB->getNumber();
      BitVector In(NumFI, !MBB->pred_empty());
      if (MBB->pred_empty())
        for (int FI = FIBase; FI < 0; ++FI)
          In.set(fiIndex(FI)); // fixed objects are defined by the caller
      for (MachineBasicBlock *P : MBB->predecessors())
        In &= Out[P->getNumber()];
      if (In != InitIn[B]) {
        InitIn[B] = In;
        Changed = true;
      }
      BitVector S = In;
      for (const MachineInstr &MI : *MBB)
        applyInit(MI, S, &Changed);
      if (S != Out[B]) {
        Out[B] = std::move(S);
        Changed = true;
      }
    }
  }
}

bool StatepointRewrite::slotInitAt(int FI, const MachineInstr *SP) {
  if (!fiInRange(FI))
    return false;
  const MachineBasicBlock *BB = SP->getParent();
  BitVector S = InitIn[BB->getNumber()];
  for (const MachineInstr &MI : *BB) {
    if (&MI == SP)
      break;
    applyInit(MI, S, nullptr);
  }
  return S.test(fiIndex(FI));
}

bool StatepointRewrite::rewriteStatepoints(MachineFunction &MF) {
  bool Changed = false;
  SmallVector<MachineInstr *, 8> WorkList;

  // Setup observed registers lists.
  ObservedRegistersWithPhys = MRI->getStatePointObservedActiveRegs();

  ObservedStackSlots.clear();
  MachineFrameInfo *MFI = &MF.getFrameInfo();
  for (int i = 0, e = MFI->getObjectIndexEnd(); i != e; ++i) {
    if (MFI->isStatepointSpillSlotObjectIndex(i))
      ObservedStackSlots.emplace_back(i);
  }

  // BUGFIX llvm23-slot-live-through-sibling: group the virtual registers by
  // their pre-split original and attach the whole family to the slot the
  // family spills to. See AddStackSlot in rewriteStatepoint() for why.
  SlotSiblingRegs.clear();
  {
    DenseMap<Register, SmallVector<Register, 4>> ByOriginal;
    for (unsigned I = 0, E = MRI->getNumVirtRegs(); I != E; ++I) {
      Register V = Register::index2VirtReg(I);
      if (MRI->reg_nodbg_empty(V) && !LIS->hasInterval(V))
        continue;
      ByOriginal[VRM->getOriginal(V)].push_back(V);
    }
    for (unsigned I = 0, E = MRI->getNumVirtRegs(); I != E; ++I) {
      Register V = Register::index2VirtReg(I);
      const int FI = VRM->getStackSlot(V);
      if (FI == VirtRegMap::NO_STACK_SLOT ||
          !MFI->isStatepointSpillSlotObjectIndex(FI))
        continue;
      auto &Sibs = SlotSiblingRegs[FI];
      for (Register S : ByOriginal[VRM->getOriginal(V)])
        if (!llvm::is_contained(Sibs, S))
          Sibs.push_back(S);
    }
  }

  // BUGFIX llvm23-statepoint-undef-slot: a value that the register allocator
  // considers dead at a split point is defined with an IMPLICIT_DEF by
  // SplitEditor::defFromParent() (LLVM 10 emitted a COPY there, because it
  // passed a full lane mask whenever the register had no subranges; LLVM 23
  // asks getLiveLaneMaskAt() and gets an empty mask for a dead parent).
  //
  // The spiller still stores such a register into its stack slot, and that slot
  // is reported to the Dart GC by the stack map below, so the collector ends up
  // scanning whatever the allocator left in the physical register -- in
  // practice unboxed payloads such as 0x7974697669746365 ("ytivitce") or a raw
  // native pointer, which makes StackFrame::VisitObjectPointers crash.
  //
  // Rewrite those IMPLICIT_DEFs into an explicit zero. The value stays
  // semantically undefined (nothing reads it), but the GC now sees the Smi 0.
  {
    const TargetInstrInfo *TII = MF.getSubtarget().getInstrInfo();
    DenseSet<int> ObsFI(ObservedStackSlots.begin(), ObservedStackSlots.end());
    SmallVector<Register, 16> RegWL;
    DenseSet<Register> RegSeen;
    auto Push = [&](Register R) {
      if (R.isVirtual() && RegSeen.insert(R).second)
        RegWL.push_back(R);
    };
    // Values that reach a GC-reported frame slot...
    if (!ObsFI.empty())
      for (MachineBasicBlock &MBB : MF)
        for (MachineInstr &MI : MBB) {
          if (!MI.mayStore())
            continue;
          bool ToObserved = false;
          for (const MachineMemOperand *MMO : MI.memoperands()) {
            const PseudoSourceValue *PSV = MMO->getPseudoValue();
            if (PSV && PSV->kind() == PseudoSourceValue::FixedStack &&
                ObsFI.count(
                    cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex()))
              ToObserved = true;
          }
          if (!ToObserved)
            continue;
          for (const MachineOperand &MO : MI.operands())
            if (MO.isReg() && MO.isUse())
              Push(MO.getReg());
        }
    // ...and values reported directly in a register.
    for (Register R : ObservedRegistersWithPhys)
      Push(R);

    // Widened: walking only the *virtual* COPY chain from a store into an
    // observed slot misses every undefined value that reaches the slot through
    // a physical-register copy, through PHI elimination, or through a
    // spill/reload pair. Those are exactly the paths that put 0x3/0xb/0x75 in a
    // reported slot. An IMPLICIT_DEF is undefined by definition, so turning any
    // GPR one into an explicit zero is always semantically valid -- and zero is
    // a Smi, which the collector skips.
    for (MachineBasicBlock &MBB : MF)
      for (MachineInstr &MI : MBB)
        if (MI.isImplicitDef() && MI.getNumOperands() == 1 &&
            MI.getOperand(0).isReg() && MI.getOperand(0).getReg().isVirtual())
          Push(MI.getOperand(0).getReg());

    while (!RegWL.empty()) {
      Register R = RegWL.pop_back_val();
      for (MachineInstr &D : MRI->def_instructions(R)) {
        if (D.isImplicitDef()) {
          if (TII->makeImplicitDefGCSafe(D)) {
            Changed = true;
          }
          continue;
        }
        if (D.isCopy() || D.isSubregToReg())
          for (const MachineOperand &MO : D.operands())
            if (MO.isReg() && MO.isUse())
              Push(MO.getReg());
      }
    }
  }

  // BUGFIX llvm23-statepoint-mustreach: only report a slot the current path has
  // definitely written.
  //
  // LiveStacks is deliberately a *may*-reach approximation (it has to be: the
  // machine verifier and StackSlotColoring consume it, and a PHI self-copy
  // legitimately loads a slot that is undefined on one edge). But the stack map
  // must not inherit that slack. Measured on device with a recognisable
  // prologue fill: at one safepoint in _Future._propagateToListeners, 21 of 63
  // reported roots still held the fill -- a third of the "live" roots had never
  // been written on that path -- and the remaining bad roots were slots holding
  // dead non-pointer values (0x3, 0xb, 0x75), which is what crashes
  // ObjectPointerVisitor::VisitPointer on a near-null address.
  //
  // So run the must-reach dataflow here, over stores only, and use it purely to
  // decide whether a slot goes into the stack map. A slot no store reaches at
  // all is dropped; a slot some path wrote is always reported, and
  // llvm23-undef-entry-init below guarantees it is defined on the other paths.
  // BUGFIX llvm23-mustreach-fixpoint: "a store to this slot reaches this point
  // on every path" is an AND-confluence (must) dataflow, i.e. the same shape as
  // available expressions, so it has to be solved for the GREATEST fixed point:
  // every block starts out "written" and the iteration only ever clears bits.
  //
  // Seeding Out from HasStore instead computes the least fixed point, which is
  // wrong for every loop. In
  //
  //     entry: store slot            latch: (no store)
  //     head:  statepoint            preds(head) = {entry, latch}
  //
  // Out[latch] starts false, so In[head] = Out[entry] && Out[latch] = false,
  // which clears Out[head], which clears In[latch] -- the whole loop settles on
  // "never written" even though the preheader writes the slot before every
  // iteration. That misclassification is what made a plain must-reach gate look
  // like it had to drop live roots, and what llvm23-undef-edge-init was
  // papering over by storing a Smi 0 on each frontier edge.
  //
  // Blocks with no predecessors (the entry block, and anything unreachable) are
  // pinned to "not written on entry".
  const unsigned NumBlocksTotal = MF.getNumBlockIDs();
  ReversePostOrderTraversal<MachineFunction *> RPOT(&MF);
  auto ComputeMustReach = [&](const BitVector &HasStore) {
    BitVector In(NumBlocksTotal), Out(NumBlocksTotal);
    In.set();
    Out.set();
    bool Changed2 = true;
    while (Changed2) {
      Changed2 = false;
      for (MachineBasicBlock *MBB : RPOT) {
        const unsigned B = MBB->getNumber();
        bool NewIn = !MBB->pred_empty();
        for (MachineBasicBlock *P : MBB->predecessors())
          if (!Out.test(P->getNumber())) {
            NewIn = false;
            break;
          }
        if (NewIn != In.test(B)) {
          In[B] = NewIn;
          Changed2 = true;
        }
        const bool NewOut = NewIn || HasStore.test(B);
        if (NewOut != Out.test(B)) {
          Out[B] = NewOut;
          Changed2 = true;
        }
      }
    }
    return In;
  };

  DefinitelyWritten.clear();
  if (!ObservedStackSlots.empty()) {
    const unsigned NumBlocks = MF.getNumBlockIDs();
    for (int FI : ObservedStackSlots) {
      BitVector HasStore(NumBlocks), In(NumBlocks);
      for (MachineBasicBlock &MBB : MF)
        for (MachineInstr &MI : MBB) {
          if (!MI.mayStore())
            continue;
          for (const MachineMemOperand *MMO : MI.memoperands()) {
            const PseudoSourceValue *PSV = MMO->getPseudoValue();
            if (PSV && PSV->kind() == PseudoSourceValue::FixedStack &&
                cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex() == FI)
              HasStore.set(MBB.getNumber());
          }
        }
      if (HasStore.none())
        continue;
      In = ComputeMustReach(HasStore);
      DefinitelyWritten[FI] = std::move(In);
      BlockHasStore[FI] = std::move(HasStore);
    }
  }

  // BUGFIX llvm23-undef-entry-init (replaces llvm23-undef-edge-init).
  //
  // The write-back below refuses to report a slot that no store must-reach,
  // because that is what an undef input looks like after the gc operands have
  // been stripped. Dropping is only sound when the slot is undef on *every*
  // path; when one path stored a real pointer and another stored nothing,
  // dropping loses a live root.
  //
  // With the must-reach dataflow solved correctly that situation has not been
  // observed (vmservice: 46368/46368 records reported, 0 drops; see
  // llvm23-mustreach-fixpoint), but it is not ruled out by construction. So
  // keep a fallback, and make it the cheapest one that is obviously correct:
  // define the slot once at the top of the entry block. The entry block
  // dominates every statepoint, so a single store makes must-reach hold for the
  // whole function -- no frontier to compute, no per-edge live interval
  // surgery, and the store is Smi 0, which the collector skips.
  if (!DefinitelyWritten.empty()) {
    const TargetInstrInfo *TIIz = MF.getSubtarget().getInstrInfo();
    const TargetRegisterInfo *TRIz = MF.getSubtarget().getRegisterInfo();
    const Register Zero = TIIz->getGCSafeZeroReg();
    MachineBasicBlock &Entry = MF.front();
    for (int FI : ObservedStackSlots) {
      auto DWIt = DefinitelyWritten.find(FI);
      if (DWIt == DefinitelyWritten.end())
        continue; // no store anywhere: pure undef, the drop is sound.
      BitVector &In = DWIt->second;
      BitVector &HasStore = BlockHasStore[FI];
      LiveInterval &SI = LSS->getInterval(FI);
      bool Need = false;
      for (MachineBasicBlock &MBB : MF) {
        for (MachineInstr &MI : MBB) {
          if (MI.getOpcode() != TargetOpcode::STATEPOINT)
            continue;
          if (!SI.liveAt(Indexes->getInstructionIndex(MI).getNextSlot()))
            continue;
          if (mustBeWrittenAt(FI, &MI))
            continue;
          Need = true;
          break;
        }
        if (Need)
          break;
      }
      if (!Need)
        continue;
      MachineBasicBlock::iterator IP =
          Entry.SkipPHIsLabelsAndDebug(Entry.begin());
      MachineBasicBlock::iterator Before =
          IP == Entry.begin() ? Entry.end() : std::prev(IP);
      TIIz->storeRegToStackSlot(Entry, IP, Zero, false, FI,
                                TRIz->getMinimalPhysRegClass(Zero), Register());
      MachineBasicBlock::iterator NewI =
          Before == Entry.end() ? Entry.begin() : std::next(Before);
      for (; NewI != IP; ++NewI) {
        Indexes->insertMachineInstrInMaps(*NewI);
        SlotIndex Idx = Indexes->getInstructionIndex(*NewI).getRegSlot();
        if (!SI.liveAt(Idx))
          SI.addSegment(LiveInterval::Segment(Idx, Idx.getDeadSlot(),
                                              SI.getValNumInfo(0)));
      }
      HasStore.set(Entry.getNumber());
      In = ComputeMustReach(HasStore);
      Changed = true;
    }
  }

  // BUGFIX llvm23-init-through-copies: "a store to the slot must-reaches the
  // statepoint" is not enough once a slot can be reported through a register
  // sibling (llvm23-slot-live-through-sibling): the sibling may be live before
  // the value was ever spilled, and a spilled loop PHI cycle can consist of
  // stores that only copy another uninitialised slot. Track initialisation
  // through values instead -- a reload from an uninitialised slot yields an
  // uninitialised register and storing it does not define the destination --
  // and report a slot only where it is initialised on every path.
  TII = MF.getSubtarget().getInstrInfo();
  FIBase = MFI->getObjectIndexBegin();
  NumFI = static_cast<unsigned>(MFI->getObjectIndexEnd() - FIBase);
  SlotHasStore.clear();
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB) {
      if (!MI.mayStore())
        continue;
      for (const MachineMemOperand *MMO : MI.memoperands()) {
        const PseudoSourceValue *PSV = MMO->getPseudoValue();
        if (PSV && PSV->kind() == PseudoSourceValue::FixedStack)
          SlotHasStore.insert(
              cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex());
      }
    }
  solveInit(MF);

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      switch (MI.getOpcode()) {
      case TargetOpcode::STATEPOINT:
        WorkList.emplace_back(&MI);
        break;
      }
    }
  }
  for (MachineInstr *MI : WorkList)
    Changed |= rewriteStatepoint(MF, MI);
  return Changed;
}

bool StatepointRewrite::rewriteStatepoint(MachineFunction &MF,
                                          MachineInstr *StatePoint) {
  MachineFrameInfo *MFI = &MF.getFrameInfo();
  // Ready to rebuild the StatePoint.
  const TargetSubtargetInfo &STI = MF.getSubtarget();
  const TargetInstrInfo *TII = STI.getInstrInfo();
  MachineInstr *NewMI = MF.CreateMachineInstr(TII->get(StatePoint->getOpcode()),
                                              StatePoint->getDebugLoc(), true);
  MachineInstrBuilder MIB(MF, NewMI);

  // Check Any Stack Slot overlaps this StatePoint.
  SlotIndex Index = Indexes->getInstructionIndex(*StatePoint);
  Index = Index.getNextSlot();
  DenseSet<int> RecordedStackSlot;

  // BUGFIX llvm23-statepoint-sections: collect the records first instead of
  // appending them to the end of the operand list. LLVM 23 locates the alloca
  // and gc map sections by walking NumGCPtrs records, so re-attached pointers
  // must go *inside* the gc pointer section with NumGCPtrs updated to match;
  // operands appended after the regmask would desync every later index.
  struct GCRecord {
    bool IsSlot;
    int SpillSize;
    int StackSlot;
    // BUGFIX llvm23-register-records: the *virtual* register. VirtRegRewriter
    // runs right after this pass and substitutes the assigned physical
    // register; emitting the physical register here would be a use of a
    // register whose units have no live range yet, which the verifier rejects
    // ("No live segment at use") -- the reason the LLVM 23 port had added the
    // register-unit guard that silently dropped every record.
    Register Reg;
  };
  SmallVector<GCRecord, 8> NewRecords;

  const TargetInstrInfo *TII2 = MF.getSubtarget().getInstrInfo();
  (void)TII2;
  auto AddStackSlot = [&](int StackSlot) {
    LiveInterval &Interval = LSS->getInterval(StackSlot);
    bool Live = Interval.liveAt(Index);
    // BUGFIX llvm23-slot-live-through-sibling: LiveStacks only merges the
    // intervals of the registers InlineSpiller actually spilled. When greedy
    // splits a value around a cold call block ("lr-split COPY" into a sibling
    // that stays in a register across the call) and copies it back
    // afterwards, the slot's interval has a hole across that call, yet the
    // spiller still reloads from the slot later, because the copy-back is the
    // same value and eliminateRedundantSpills() found it already spilled. For
    // a plain value that is fine; for a gc pointer the object may move at the
    // call, the register copy is updated (shared stub push / CSR slots) but
    // the unreported slot keeps the old address, and the later reload hands
    // the mutator a stale pointer (JpegEncoder.encode, slot %stack.7 across
    // mock_call_CheckStackOverflow -> SEGV_ACCERR in the freed semispace).
    // The slot is a root for as long as any sibling of the family is live.
    if (!Live) {
      auto It = SlotSiblingRegs.find(StackSlot);
      if (It != SlotSiblingRegs.end())
        for (Register S : It->second)
          if (LIS->hasInterval(S) && LIS->getInterval(S).liveAt(Index)) {
            Live = true;
            ++DartSiblingLiveSlots;
            break;
          }
      if (Live && Interval.getNumValNums() > 0) {
        // Keep LiveStacks consistent with the record (the verifier checks
        // the statepoint's memoperand against it, and StackSlotColoring must
        // not hand the slot to another value across this call).
        const SlotIndex SPIdx = Indexes->getInstructionIndex(*StatePoint);
        Interval.addSegment(LiveInterval::Segment(SPIdx.getBaseIndex(),
                                                  SPIdx.getDeadSlot(),
                                                  Interval.getValNumInfo(0)));
      }
    }
    if (!Live)
      return;
    // HISTORY(llvm23-statepoint-mustreach): a plain must-reach gate here does
    // remove the bogus roots, but on its own it also drops roots that are live
    // on the path actually taken, which is fatal for async -- Dart copies a
    // suspended frame into a _SuspendState and re-scans it with this same stack
    // map, so an unreported pointer never gets updated when its object moves
    // (crash on resume in _SuspendState._createAsyncCallbacks_thenCall).
    // The rule below is must-reach made sound: suspendable frames are exempt,
    // the dataflow is solved for the greatest fixed point
    // (llvm23-mustreach-fixpoint) so loop-carried slots are no longer
    // misclassified, and llvm23-undef-entry-init defines the slot once in the
    // entry block if any statepoint could still see it undefined.
    // BUGFIX llvm23-undef-not-written-back: if the value this slot holds at
    // this statepoint is undef, do not write a gc record for it.
    //
    // This mirrors what upstream does for an undef gc value in
    // StatepointLowering::lowerIncomingStatepointValue(): it never gives undef
    // a register or a spill slot, it records a plain stack-map constant
    // (0xFEFEFEFE) instead, because "the compiler is always allowed to chose an
    // arbitrary value for undef". Our scheme strips the gc operands before RA
    // and rebuilds them here from slot liveness, so that decision has to be
    // re-made at write-back time -- otherwise a value upstream deliberately
    // kept out of every slot comes back as a slot-based root and the GC
    // dereferences whatever the frame happened to contain.
    //
    // A slot's input is undef exactly when no store reaches this point at all.
    // The register-level test is useless here: by the time this pass runs the
    // owning vreg has been spilled, so its live interval is not live at the
    // statepoint at all (measured: 55000 candidates, 0 hits).
    //
    // Dropping the record cannot lose a root: an undef value refers to no
    // object, so there is nothing to keep alive or relocate.
    //
    // The exception is a suspendable function. Dart copies a suspended frame
    // into a _SuspendState and copies it back on resume, so on the resume path
    // the slots are written by the runtime with no store instruction in this
    // function at all. A store-based analysis calls those slots undef and drops
    // live roots -- that is what made the earlier must-reach attempt crash in
    // _SuspendState._createAsyncCallbacks_thenCall. Keep reporting there.
    // BUGFIX llvm23-suspendable-uninit-slot: the exemption for suspendable
    // functions was wrong. Resume copies the frame back exactly as it was at
    // the suspension call, so every store fact that reached that call still
    // holds after it; the one slot the runtime writes without a store here --
    // the _SuspendState slot -- is force-marked by the assembler
    // (llvm_code_assembler.cc IsSuspendableFunction) and needs no record.
    // What the exemption did do was report a sibling-live slot before its
    // first store: MethodChannel._invokeMethod (async) records sp+0x28 at the
    // encodeMethodCall call while its only store, "str x10, [x15, #0x28]",
    // comes 0x30 bytes later on the other path; the previous frame's leftover
    // (0x5) went to the scavenger as a root (SEGV in VisitPointers, 2 of 3
    // cold starts). Until now the FixupStatepointCallerSaved spill of x12
    // happened to share and initialise that slot at every call, hiding it.
    {
      // Two drop cases, and they differ in soundness:
      //   no store to this slot anywhere  -> pure undef, dropping is sound;
      //   stores exist but do not must-reach -> some path carries a real
      //     pointer, so dropping would lose a root. llvm23-undef-entry-init
      //     above has already defined the slot in the entry block whenever that
      //     can happen, so reaching here with stores present means the analysis
      //     and the fallback disagree; count it and keep the record.
      // BUGFIX llvm23-init-through-copies: no store anywhere is a pure undef
      // and dropping is sound; not initialised on every path to this
      // statepoint means the slot may hold whatever the frame had, so it must
      // not be handed to the collector either.
      if (!SlotHasStore.count(StackSlot))
        return;
      if (!slotInitAt(StackSlot, StatePoint)) {
        ++DartSiblingStatsInst.Uninit;
        return;
      }
    }
    if (!RecordedStackSlot.insert(StackSlot).second)
      return;
    NewRecords.push_back(
        {true, static_cast<int>(MFI->getObjectSize(StackSlot)), StackSlot, {}});
  };

  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  auto AddPhysIfLiveOut = [&](Register Reg) {
    if (!LIS->hasInterval(Reg) || !LIS->getInterval(Reg).liveAt(Index)) {
      return false;
    }
    MCRegister Phys = VRM->getPhys(Reg);
    if (!Phys.isValid())
      return false;
    // BUGFIX llvm23-register-records: the LLVM 23 port additionally required
    // the physical register's *register units* to be live here, to guard
    // against a vreg whose def was folded away. But this pass runs before
    // VirtRegRewriter, and register-unit live ranges only know explicit
    // physical operands -- a virtual register assigned to x20 never makes
    // x20's unit live -- so the guard rejected essentially every register
    // record (JpegEncoder.encode: 0 register records at its 55 loop call
    // sites). A tagged value that the allocator keeps in a caller-saved
    // register across a register-preserving shared stub (CheckStackOverflow
    // at the loop back-edge) was then invisible to the collector, came back
    // stale when the stub returned, and was spilled stale into a slot every
    // later record faithfully "updated" (SEGV_ACCERR in the freed semispace
    // at JpegEncoder.encode+0x1af8). Ask the virtual register's own live
    // range instead: it is live here and has a real, used value number.
    const VNInfo *VNI = LIS->getInterval(Reg).getVNInfoAt(Index);
    if (!VNI || VNI->isUnused())
      return false;
    ++DartSiblingStatsInst.RegRecs;
    NewRecords.push_back({false, 0, 0, Reg});
    return true;
  };

  // Add Stack Slots
  for (int StackSlot : ObservedStackSlots) {
    AddStackSlot(StackSlot);
  }

  for (Register r : ObservedRegistersWithPhys) {
    AddPhysIfLiveOut(r);
  }

  StatepointOpers SO(StatePoint);
  const unsigned NumGCPtrIdx = SO.getNumGCPtrIdx();
  const unsigned AllocaMarkerIdx = SO.getNumAllocaIdx() - 1;
  const unsigned NumGcMapIdx = SO.getNumGcMapEntriesIdx();
  const int64_t NumGCPtrs = StatePoint->getOperand(NumGCPtrIdx).getImm();
  const int64_t OldGcMapEntries = StatePoint->getOperand(NumGcMapIdx).getImm();
  const int64_t TotalGCPtrs =
      NumGCPtrs + static_cast<int64_t>(NewRecords.size());

  for (unsigned i = 0, E = StatePoint->getNumOperands(); i < E; ++i) {
    if (i == AllocaMarkerIdx) {
      for (const GCRecord &R : NewRecords) {
        if (R.IsSlot) {
          MIB.addImm(StackMaps::IndirectMemRefOp);
          MIB.addImm(R.SpillSize);
          MIB.addFrameIndex(R.StackSlot);
          MIB.addImm(0);
        } else {
          MIB.addReg(R.Reg);
        }
      }
    }
    if (i == NumGCPtrIdx) {
      MIB.addImm(TotalGCPtrs);
      continue;
    }
    if (i == NumGcMapIdx) {
      // BUGFIX llvm23-statepoint-gcmap: StackMaps::parseStatepointOpers() emits
      // a location for a gc pointer ONLY by walking the base/derived pairs in
      // the gc map section (see the `for (auto &P : GCPairs)` loop) -- it never
      // iterates the gc pointer list itself. removeVarFromStatepoint() drops
      // the original map because its indices refer to pointers that are gone,
      // so unless the map is rebuilt here every re-attached record is silently
      // dropped from .llvm_stackmaps and the GC never learns about the spill
      // slot. Dart has no derived pointers, so emit one identity pair per gc
      // pointer.
      MIB.addImm(TotalGCPtrs);
      for (int64_t K = 0; K < TotalGCPtrs; ++K) {
        MIB.addImm(K);
        MIB.addImm(K);
      }
      i += static_cast<unsigned>(2 * OldGcMapEntries);
      continue;
    }
    MIB.add(StatePoint->getOperand(i));
  }

  // BUGFIX llvm23-statepoint-sections: a frame index referenced from the
  // stackmap section needs a matching memoperand, or the machine verifier
  // reports "Missing fixed stack memoperand" (same bookkeeping as
  // FixupStatepointCallerSaved::rewriteStatepoint).
  NewMI->setMemRefs(MF, StatePoint->memoperands());
  for (const GCRecord &R : NewRecords) {
    if (!R.IsSlot)
      continue;
    auto PtrInfo = MachinePointerInfo::getFixedStack(MF, R.StackSlot);
    auto *MMO =
        MF.getMachineMemOperand(PtrInfo, MachineMemOperand::MOLoad, R.SpillSize,
                                MFI->getObjectAlign(R.StackSlot));
    NewMI->addMemOperand(MF, MMO);
  }

  MachineBasicBlock *MBB = StatePoint->getParent();
  MachineBasicBlock::iterator Pos = StatePoint;
  MBB->insert(Pos, NewMI);
  LIS->ReplaceMachineInstrInMaps(*StatePoint, *NewMI);
  MBB->erase(StatePoint);
  return true;
}

FunctionPass *llvm::createStatepointSimplifyPass() {
  return new StatepointSimplify();
}

FunctionPass *llvm::createStatepointRewritePass() {
  return new StatepointRewrite();
}
