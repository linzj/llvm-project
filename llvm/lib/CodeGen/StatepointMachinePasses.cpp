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
#include "llvm/ADT/BitVector.h"
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
  bool rewriteStatepoint(MachineFunction &, MachineInstr *MI,
                         const BitVector &LiveSlots);

  SmallVector<Register, 8> ObservedRegistersWithPhys;
  SmallVector<int, 8> ObservedStackSlots;
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

bool StatepointRewrite::rewriteStatepoints(MachineFunction &MF) {
  bool Changed = false;
  SmallVector<std::pair<MachineInstr *, BitVector>, 8> WorkList;

  // Setup observed registers lists.
  ObservedRegistersWithPhys = MRI->getStatePointObservedActiveRegs();

  ObservedStackSlots.clear();
  MachineFrameInfo *MFI = &MF.getFrameInfo();
  for (int i = 0, e = MFI->getObjectIndexEnd(); i != e; ++i) {
    if (MFI->isStatepointSpillSlotObjectIndex(i))
      ObservedStackSlots.emplace_back(i);
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

  // HISTORY: the must-reach dataflow (llvm23-statepoint-mustreach,
  // llvm23-mustreach-fixpoint), the entry-block Smi-0 store
  // (llvm23-undef-entry-init) and the value-aware initialisation dataflow
  // (llvm23-init-through-copies) that used to live here all compensated for
  // the IMPLICIT_DEF SplitEditor::defFromParent() emitted for a dead parent
  // value. That is now closed at the source (llvm23-split-observed-full-copy
  // in SplitKit.cpp): an observed value is always defined with a real COPY at
  // a split point, so a slot whose contents are still needed has been written
  // on every path and no fallback stores are required. The MustInit/MayRead
  // dataflow below (llvm23-statepoint-slot-liveness) does a different job: it
  // decides at each statepoint whether the slot's current contents are still
  // a live root.

  // BUGFIX llvm23-statepoint-slot-liveness: report an observed spill slot at a
  // statepoint iff every reachable path from entry has written the full slot
  // (MustInit) and some path after the statepoint reads the slot before it is
  // overwritten (MayRead). Tracking sibling virtual registers instead falsely
  // resurrected dead slots whose register was redefined without re-spilling.
  if (ObservedStackSlots.empty()) {
    for (MachineBasicBlock &MBB : MF)
      for (MachineInstr &MI : MBB)
        if (MI.getOpcode() == TargetOpcode::STATEPOINT)
          WorkList.emplace_back(&MI, BitVector());
  } else {
    const unsigned NumSlots = ObservedStackSlots.size();
    DenseMap<int, unsigned> SlotToBit;
    SlotToBit.reserve(NumSlots);
    for (unsigned Bit = 0; Bit < NumSlots; ++Bit)
      SlotToBit[ObservedStackSlots[Bit]] = Bit;

    const TargetInstrInfo *TII = MF.getSubtarget().getInstrInfo();
    const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

    auto ClassifyMI = [&](const MachineInstr &MI, BitVector &ReadBits,
                          BitVector &FullWriteBits) {
      ReadBits.reset();
      FullWriteBits.reset();
      if (MI.isDebugInstr() || MI.isKill() || MI.isImplicitDef() ||
          MI.getOpcode() == TargetOpcode::STATEPOINT)
        return;

      int LoadFI = 0;
      if (TII->isLoadFromStackSlot(MI, LoadFI)) {
        auto It = SlotToBit.find(LoadFI);
        if (It != SlotToBit.end())
          ReadBits.set(It->second);
        return;
      }

      int StoreFI = 0;
      TypeSize StoreBytes = TypeSize::getZero();
      if (Register StoreReg =
              TII->isStoreToStackSlot(MI, StoreFI, StoreBytes)) {
        auto It = SlotToBit.find(StoreFI);
        if (It != SlotToBit.end() && !TII->isPredicated(MI)) {
          const uint64_t ObjSize =
              static_cast<uint64_t>(MFI->getObjectSize(StoreFI));
          uint64_t Written = 0;
          if (!StoreBytes.isZero() && !StoreBytes.isScalable()) {
            Written = StoreBytes.getFixedValue();
          } else {
            for (const MachineMemOperand *MMO : MI.memoperands()) {
              const PseudoSourceValue *PSV = MMO->getPseudoValue();
              if (PSV && PSV->kind() == PseudoSourceValue::FixedStack &&
                  cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex() ==
                      StoreFI &&
                  MMO->getOffset() == 0) {
                const LLT MemTy = MMO->getType();
                if (MemTy.isValid() && !MemTy.isScalable()) {
                  Written = MemTy.getSizeInBytes().getFixedValue();
                  break;
                }
              }
            }
            if (Written == 0 && StoreReg.isValid()) {
              if (StoreReg.isVirtual())
                Written = TRI->getSpillSize(*MRI->getRegClass(StoreReg));
              else if (const TargetRegisterClass *RC =
                           TRI->getMinimalPhysRegClass(StoreReg))
                Written = TRI->getSpillSize(*RC);
            }
          }
          if (Written >= ObjSize)
            FullWriteBits.set(It->second);
        }
        return;
      }

      bool SawMMO = false;
      for (const MachineMemOperand *MMO : MI.memoperands()) {
        const PseudoSourceValue *PSV = MMO->getPseudoValue();
        if (!PSV || PSV->kind() != PseudoSourceValue::FixedStack)
          continue;
        SawMMO = true;
        const int FI = cast<FixedStackPseudoSourceValue>(PSV)->getFrameIndex();
        auto It = SlotToBit.find(FI);
        if (It == SlotToBit.end())
          continue;
        // Classify by the MMO itself, not by MI.mayLoad()/mayStore(): one
        // instruction can carry a load MMO for one slot and a store MMO for
        // another (e.g. SystemZ folds a spilled load or store into a
        // memory-to-memory MVC), and the store side must still count as a
        // full write. A load+store MMO (read-modify-write) is only a read.
        if (MMO->isLoad())
          ReadBits.set(It->second);
        if (MMO->isStore() && !MMO->isLoad() && !TII->isPredicated(MI)) {
          const uint64_t ObjSize =
              static_cast<uint64_t>(MFI->getObjectSize(FI));
          const LLT MemTy = MMO->getType();
          const uint64_t Sz = (MemTy.isValid() && !MemTy.isScalable())
                                  ? MemTy.getSizeInBytes().getFixedValue()
                                  : 0;
          const int64_t Off = MMO->getOffset();
          if (Off <= 0 && Sz >= ObjSize + static_cast<uint64_t>(-Off))
            FullWriteBits.set(It->second);
        }
      }
      if (!SawMMO) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI())
            continue;
          auto It = SlotToBit.find(MO.getIndex());
          if (It == SlotToBit.end())
            continue;
          if (MI.mayLoad())
            ReadBits.set(It->second);
          else if (MI.mayStore() && !TII->isPredicated(MI))
            FullWriteBits.set(It->second);
        }
      }
    };

    const unsigned MaxBB = MF.getNumBlockIDs();
    BitVector Reachable(MaxBB);
    if (!MF.empty()) {
      SmallVector<const MachineBasicBlock *, 16> RQueue;
      Reachable.set(MF.front().getNumber());
      RQueue.push_back(&MF.front());
      while (!RQueue.empty()) {
        const MachineBasicBlock *Cur = RQueue.pop_back_val();
        for (const MachineBasicBlock *Succ : Cur->successors()) {
          if (!Reachable.test(Succ->getNumber())) {
            Reachable.set(Succ->getNumber());
            RQueue.push_back(Succ);
          }
        }
      }
    }

    std::vector<BitVector> BlockGenInit(MaxBB, BitVector(NumSlots));
    std::vector<BitVector> BlockGenRead(MaxBB, BitVector(NumSlots));
    std::vector<BitVector> BlockKillWrite(MaxBB, BitVector(NumSlots));
    BitVector ReadBits(NumSlots), FullWriteBits(NumSlots);

    for (MachineBasicBlock &MBB : MF) {
      const unsigned B = MBB.getNumber();
      if (!Reachable.test(B))
        continue;
      BitVector &GenInit = BlockGenInit[B];
      BitVector &GenRead = BlockGenRead[B];
      BitVector &KillWrite = BlockKillWrite[B];
      for (const MachineInstr &MI : llvm::reverse(MBB)) {
        ClassifyMI(MI, ReadBits, FullWriteBits);
        GenInit |= FullWriteBits;
        GenRead.reset(FullWriteBits);
        KillWrite |= FullWriteBits;
        GenRead |= ReadBits;
        KillWrite.reset(ReadBits);
      }
    }

    // Forward greatest fixed point: MustInit.
    const BitVector AllOnes(NumSlots, true);
    std::vector<BitVector> InitIn(MaxBB, BitVector(NumSlots));
    std::vector<BitVector> InitOut(MaxBB, AllOnes);
    if (!MF.empty()) {
      const unsigned EntryB = MF.front().getNumber();
      InitOut[EntryB] = BlockGenInit[EntryB];
      bool IterChanged = true;
      BitVector NewIn(NumSlots), NewOut(NumSlots);
      while (IterChanged) {
        IterChanged = false;
        for (MachineBasicBlock &MBB : MF) {
          const unsigned B = MBB.getNumber();
          if (!Reachable.test(B) || &MBB == &MF.front())
            continue;
          NewIn = AllOnes;
          for (const MachineBasicBlock *Pred : MBB.predecessors()) {
            if (Reachable.test(Pred->getNumber()))
              NewIn &= InitOut[Pred->getNumber()];
          }
          InitIn[B] = NewIn;
          NewOut = NewIn;
          NewOut |= BlockGenInit[B];
          if (NewOut != InitOut[B]) {
            InitOut[B] = NewOut;
            IterChanged = true;
          }
        }
      }
    }

    // Backward least fixed point: MayRead before full overwrite.
    std::vector<BitVector> MayReadIn(MaxBB, BitVector(NumSlots));
    std::vector<BitVector> MayReadOut(MaxBB, BitVector(NumSlots));
    {
      bool IterChanged = true;
      BitVector NewOut(NumSlots), NewIn(NumSlots);
      while (IterChanged) {
        IterChanged = false;
        for (MachineBasicBlock &MBB : llvm::reverse(MF)) {
          const unsigned B = MBB.getNumber();
          if (!Reachable.test(B))
            continue;
          NewOut.reset();
          for (const MachineBasicBlock *Succ : MBB.successors()) {
            if (Reachable.test(Succ->getNumber()))
              NewOut |= MayReadIn[Succ->getNumber()];
          }
          MayReadOut[B] = NewOut;
          NewIn = NewOut;
          NewIn.reset(BlockKillWrite[B]);
          NewIn |= BlockGenRead[B];
          if (NewIn != MayReadIn[B]) {
            MayReadIn[B] = NewIn;
            IterChanged = true;
          }
        }
      }
    }

    // Collect STATEPOINTs with their exact live slot bitvectors.
    for (MachineBasicBlock &MBB : MF) {
      bool HasSP = false;
      for (MachineInstr &MI : MBB) {
        if (MI.getOpcode() == TargetOpcode::STATEPOINT) {
          HasSP = true;
          break;
        }
      }
      if (!HasSP)
        continue;

      const unsigned B = MBB.getNumber();
      if (!Reachable.test(B)) {
        for (MachineInstr &MI : MBB)
          if (MI.getOpcode() == TargetOpcode::STATEPOINT)
            WorkList.emplace_back(&MI, BitVector(NumSlots));
        continue;
      }

      DenseMap<const MachineInstr *, BitVector> InitAtSP;
      BitVector CurInit = InitIn[B];
      for (MachineInstr &MI : MBB) {
        if (MI.getOpcode() == TargetOpcode::STATEPOINT) {
          InitAtSP.try_emplace(&MI, CurInit);
        } else {
          ClassifyMI(MI, ReadBits, FullWriteBits);
          CurInit |= FullWriteBits;
        }
      }

      SmallVector<std::pair<MachineInstr *, BitVector>, 4> BlockSPs;
      BitVector CurMayRead = MayReadOut[B];
      for (MachineInstr &MI : llvm::reverse(MBB)) {
        if (MI.getOpcode() == TargetOpcode::STATEPOINT) {
          BitVector LiveAtSP = InitAtSP.lookup(&MI);
          LiveAtSP &= CurMayRead;
          BlockSPs.emplace_back(&MI, std::move(LiveAtSP));
        } else {
          ClassifyMI(MI, ReadBits, FullWriteBits);
          CurMayRead.reset(FullWriteBits);
          CurMayRead |= ReadBits;
        }
      }
      for (auto &Entry : llvm::reverse(BlockSPs))
        WorkList.push_back(std::move(Entry));
    }
  }

  for (auto &Entry : WorkList)
    Changed |= rewriteStatepoint(MF, Entry.first, Entry.second);
  return Changed;
}

bool StatepointRewrite::rewriteStatepoint(MachineFunction &MF,
                                          MachineInstr *StatePoint,
                                          const BitVector &LiveSlots) {
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

  for (unsigned Bit = 0, E = ObservedStackSlots.size(); Bit != E; ++Bit) {
    if (!LiveSlots.test(Bit))
      continue;
    const int StackSlot = ObservedStackSlots[Bit];
    if (!RecordedStackSlot.insert(StackSlot).second)
      continue;
    if (LSS->hasInterval(StackSlot)) {
      LiveInterval &Interval = LSS->getInterval(StackSlot);
      if (!Interval.liveAt(Index) && Interval.getNumValNums() > 0) {
        const SlotIndex SPIdx = Indexes->getInstructionIndex(*StatePoint);
        Interval.addSegment(LiveInterval::Segment(SPIdx.getBaseIndex(),
                                                  SPIdx.getDeadSlot(),
                                                  Interval.getValNumInfo(0)));
      }
    }
    NewRecords.push_back(
        {true, static_cast<int>(MFI->getObjectSize(StackSlot)), StackSlot, {}});
  }

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
    NewRecords.push_back({false, 0, 0, Reg});
    return true;
  };

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
