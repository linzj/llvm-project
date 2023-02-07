#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineOptimizationRemarkEmitter.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>
#include <cstdint>
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "frame-elide"

namespace {

/// Class to determine where the safe point to insert the
/// prologue and epilogue are.
/// Unlike the paper from Fred C. Chow, PLDI'88, that introduces the
/// shrink-wrapping term for prologue/epilogue placement, this pass
/// does not rely on expensive data-flow analysis. Instead we use the
/// dominance properties and loop information to decide which point
/// are safe for such insertion.
class FrameElide : public MachineFunctionPass {
  /// The information about a block's conditional terminators needed to trace
  /// our predicate state through the exiting edges.
  struct BlockCondInfo {
    MachineBasicBlock *MBB;

    // We mostly have one conditional branch, and in extremely rare cases have
    // two. Three and more are so rare as to be unimportant for compile time.
    SmallVector<MachineInstr *, 2> CondBrs;

    MachineInstr *UncondBr;

    int JumpTableIndex;
  };
  /// Hold the information of the branch probability.
  /// Use to check the profitability of the new points.
  const MachineBranchProbabilityInfo *MBPI;

  /// Hold the loop information. Used to determine if Save and Restore
  /// are in the same loop.
  MachineLoopInfo *MLI;

  // Emit remarks.
  MachineOptimizationRemarkEmitter *ORE = nullptr;
  // Machine Frame Info.
  MachineFrameInfo *MFI = nullptr;

  /// Current opcode for frame setup.
  unsigned FrameSetupOpcode;

  /// Current opcode for frame destroy.
  unsigned FrameDestroyOpcode;

  /// Stack pointer register, used by llvm.{savestack,restorestack}
  unsigned SP;

  /// Entry block.
  const MachineBasicBlock *Entry;

  using SetOfRegs = SmallSetVector<unsigned, 16>;

  /// Registers that need to be saved for the current function.
  mutable SetOfRegs CurrentCSRs;

  /// Current MachineFunction.
  MachineFunction *MachineFunc;

  const TargetInstrInfo *TII;
  std::unique_ptr<RegScavenger> RS;
  BitVector DeferredBlocks;

  /// a frame index. If this is the case, this means \p MI must happen
  /// after Save and before Restore.
  bool useOrDefCSROrFI(const MachineInstr &MI) const;

  const SetOfRegs &getCurrentCSRs() const {
    if (CurrentCSRs.empty()) {
      BitVector SavedRegs;
      const TargetFrameLowering *TFI =
          MachineFunc->getSubtarget().getFrameLowering();

      TFI->determineCalleeSaves(*MachineFunc, SavedRegs, RS.get());

      const TargetRegisterInfo *TRI =
          MachineFunc->getSubtarget().getRegisterInfo();
      for (int Reg = SavedRegs.find_first(); Reg != -1;
           Reg = SavedRegs.find_next(Reg)) {
        for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
          CurrentCSRs.insert(*AI);
      }
    }
    return CurrentCSRs;
  }

  /// Initialize the pass for \p MF.
  void init(MachineFunction &MF) {
    MFI = &MF.getFrameInfo();
    MBPI = &getAnalysis<MachineBranchProbabilityInfo>();
    MLI = &getAnalysis<MachineLoopInfo>();
    ORE = &getAnalysis<MachineOptimizationRemarkEmitterPass>().getORE();
    const TargetSubtargetInfo &Subtarget = MF.getSubtarget();
    TII = Subtarget.getInstrInfo();
    FrameSetupOpcode = TII->getCallFrameSetupOpcode();
    FrameDestroyOpcode = TII->getCallFrameDestroyOpcode();
    SP = Subtarget.getTargetLowering()->getStackPointerRegisterToSaveRestore();
    Entry = &MF.front();
    CurrentCSRs.clear();
    MachineFunc = &MF;
    const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
    RS.reset(TRI->requiresRegisterScavenging(MF) ? new RegScavenger()
                                                 : nullptr);
  }

  bool ensureMFSplitEdgeForm();
  void splitEdge(MachineBasicBlock &MBB, MachineBasicBlock &Succ, int SuccCount,
                 MachineInstr *Br, MachineInstr **UncondBr,
                 std::vector<MachineBasicBlock *>::iterator *JumpTableIterator);
  void splitJumpTable(MachineBasicBlock &MBB, int JumpTableIndex);
  SmallVector<BlockCondInfo, 16> collectBlockCondInfo(MachineFunction &MF);
  MachineOperand &getBranchDestination(MachineInstr &MI);
  void markBlocks();
  void propagateMarks();
  bool propagateInOrder();
  bool propagateReversed();
  bool propagateIntoBlock(MachineBasicBlock &MBB);
  bool deferredBlock(MachineBasicBlock &MBB);
  void determineDeferredBlocks();

public:
  static char ID;

  FrameElide() : MachineFunctionPass(ID) {
    initializeFrameElidePass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    AU.addRequired<MachineBranchProbabilityInfo>();
    AU.addRequired<MachineBlockFrequencyInfo>();
    AU.addRequired<MachineOptimizationRemarkEmitterPass>();
    AU.addRequired<MachineLoopInfo>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoVRegs);
  }

  StringRef getPassName() const override { return "Frame elide optimization"; }

  /// Perform the shrink-wrapping analysis and update
  /// the MachineFrameInfo attached to \p MF with the results.
  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool mergeEHPad(MachineFunction &);

class EHPadNode : public FoldingSetNode {
public:
  EHPadNode(MachineBasicBlock *MBB);
  ~EHPadNode() = default;

  void AddPad(MachineBasicBlock *MBB);
  SmallVector<MachineBasicBlock *, 8> &Pads();

  void Profile(FoldingSetNodeID &ID) const;
  MachineBasicBlock *getTarget() const;

private:
  MachineBasicBlock *Target;
  SmallVector<MachineBasicBlock *, 8> EHPads;
};

EHPadNode::EHPadNode(MachineBasicBlock *MBB) : Target(MBB) {}

void EHPadNode::AddPad(MachineBasicBlock *MBB) { EHPads.push_back(MBB); }

void EHPadNode::Profile(FoldingSetNodeID &ID) const {
  ID.AddPointer(Target);
  MachineBasicBlock *Head = EHPads[0];
  auto It = Head->begin(), End = Head->end();
  ++It;
  for (; It != End; ++It) {
    if (It->isTerminator() && !It->isReturn())
      break;
    ID.AddInteger(MachineInstrExpressionTrait::getHashValue(&*It));
  }
}

SmallVector<MachineBasicBlock *, 8> &EHPadNode::Pads() { return EHPads; }

MachineBasicBlock *EHPadNode::getTarget() const { return Target; }

bool mergeEHPad(MachineFunction &MF) {
  FoldingSet<EHPadNode> EHPadNodeSet;
  BumpPtrAllocator Alloc;
  // Collect EHPad points to the same targets.
  for (MachineBasicBlock &MBB : MF) {
    if (!MBB.isEHPad())
      continue;
    if (MBB.succ_size() > 1)
      continue;
    // Validate block
    // Must be:
    // EH_LABEL <mcsymbol >
    // %2644:gpr64 = COPY $x1
    // %2643:gpr64 = COPY $x0
    // B %bb.x
    auto ValidateIt = MBB.begin(), End = MBB.end();
    if (!ValidateIt->isEHLabel())
      continue;
    ++ValidateIt;
    FoldingSetNodeID ID;
    MachineBasicBlock *Target;
    if (MBB.succ_size() == 1) {
      Target = *MBB.succ_begin();
    } else {
      Target = nullptr;
    }
    ID.AddPointer(Target);
    for (; ValidateIt != End; ++ValidateIt) {
      if (ValidateIt->isTerminator() && !ValidateIt->isReturn())
        break;
      ID.AddInteger(MachineInstrExpressionTrait::getHashValue(&*ValidateIt));
    }
    // Not fallthrough branch then must be a unconditional branch or a return.
    if (ValidateIt != End && !ValidateIt->isUnconditionalBranch())
      continue;

    void *InsertPos;
    if (EHPadNode *Existing = EHPadNodeSet.FindNodeOrInsertPos(ID, InsertPos)) {
      MachineBasicBlock *Head = Existing->Pads()[0];
      auto it0 = Head->begin();
      auto end0 = Head->end();
      // skip EH_LABEL.
      ++it0;
      auto it1 = MBB.begin();
      auto end1 = MBB.end();
      // skip EH_LABEL.
      ++it1;
      for (; it0 != end0; ++it0, ++it1) {
        if (it0->isTerminator())
          break;
        if (!it0->isIdenticalTo(*it1)) {
          goto not_identical;
        }
      }
      Existing->AddPad(&MBB);
    not_identical:
      continue;
    }
    EHPadNode *NewNode = new (Alloc) EHPadNode(Target);
    NewNode->AddPad(&MBB);
    EHPadNodeSet.InsertNode(NewNode, InsertPos);
  }

  bool Changed = false;
  // Merge pads branch to the same target.
  for (EHPadNode &Node : EHPadNodeSet) {
    auto &EHPads = Node.Pads();
    if (EHPads.size() <= 1)
      continue;
    Changed = true;
    MachineBasicBlock *MoveTo = EHPads[0];
    for (size_t i = 1; i < EHPads.size(); ++i) {
      MachineBasicBlock *From = EHPads[i];
      // Shall be:
      // EH_LABEL <mcsymbol >
      // %421:gpr64 = COPY $x1
      // %420:gpr64 = COPY $x0
      // B xxx
      MoveTo->splice(MoveTo->begin(), From, From->begin());
      // Change CFG.
      SmallVector<MachineBasicBlock *, 4> Preds(From->predecessors());
      for (auto Pred : Preds) {
        Pred->replaceSuccessor(From, MoveTo);
      }
      if (From->succ_size() == 1)
        From->removeSuccessor(Node.getTarget());
      From->eraseFromParent();
    }
  }
  return Changed;
}
} // end anonymous namespace

char FrameElide::ID = 0;

char &llvm::FrameElideID = FrameElide::ID;

INITIALIZE_PASS_BEGIN(FrameElide, DEBUG_TYPE, "Frame Elide Pass", false, false)
INITIALIZE_PASS_DEPENDENCY(MachineBranchProbabilityInfo)
INITIALIZE_PASS_DEPENDENCY(MachineBlockFrequencyInfo)
INITIALIZE_PASS_DEPENDENCY(MachineOptimizationRemarkEmitterPass)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(FrameElide, DEBUG_TYPE, "Frame Elide Pass", false, false)

bool FrameElide::useOrDefCSROrFI(const MachineInstr &MI) const {
  if (MI.getOpcode() == FrameSetupOpcode ||
      MI.getOpcode() == FrameDestroyOpcode) {
    LLVM_DEBUG(dbgs() << "Frame instruction: " << MI << '\n');
    return true;
  }
  if (MI.isReturn())
    return false;
  for (const MachineOperand &MO : MI.operands()) {
    bool UseOrDefCSR = false;
    if (MO.isReg()) {
      // Ignore instructions like DBG_VALUE which don't read/def the register.
      if (!MO.isDef() && !MO.readsReg())
        continue;
      Register PhysReg = MO.getReg();
      if (!PhysReg)
        continue;
      assert(Register::isPhysicalRegister(PhysReg) && "Unallocated register?!");
      // The stack pointer is not normally described as a callee-saved register
      // in calling convention definitions, so we need to watch for it
      // separately. An SP mentioned by a call instruction, we can ignore,
      // though, as it's harmless and we do not want to effectively disable tail
      // calls by forcing the restore point to post-dominate them.
      UseOrDefCSR =
          (!MI.isCall() && PhysReg == SP) || getCurrentCSRs().count(PhysReg);
    } else if (MO.isRegMask()) {
      // Check if this regmask clobbers any of the CSRs.
      for (unsigned Reg : getCurrentCSRs()) {
        if (MO.clobbersPhysReg(Reg)) {
          UseOrDefCSR = true;
          break;
        }
      }
    }
    // Skip FrameIndex operands in DBG_VALUE instructions.
    if (UseOrDefCSR || (MO.isFI() && !MFI->isFixedObjectIndex(MO.getIndex()) &&
                        !MI.isDebugValue())) {
      LLVM_DEBUG(dbgs() << "Use or define CSR(" << UseOrDefCSR << ") or FI("
                        << MO.isFI() << "): " << MI << '\n');
      return true;
    }
  }
  return false;
}

static bool giveUpWithRemarks(MachineOptimizationRemarkEmitter *ORE,
                              StringRef RemarkName, StringRef RemarkMessage,
                              const DiagnosticLocation &Loc,
                              const MachineBasicBlock *MBB) {
  ORE->emit([&]() {
    return MachineOptimizationRemarkMissed(DEBUG_TYPE, RemarkName, Loc, MBB)
           << RemarkMessage;
  });

  LLVM_DEBUG(dbgs() << RemarkMessage << '\n');
  return false;
}

bool FrameElide::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()) || MF.empty())
    return false;

  LLVM_DEBUG(dbgs() << "**** Analysing " << MF.getName() << '\n');

  init(MF);
  // Make sure blocks are numbered in order
  MF.RenumberBlocks();

  if (!ensureMFSplitEdgeForm())
    return mergeEHPad(MF), false;
  determineDeferredBlocks();
  markBlocks();
  propagateMarks();

  return mergeEHPad(MF);
}

bool FrameElide::ensureMFSplitEdgeForm() {
  auto Infos = collectBlockCondInfo(*MachineFunc);
  if (Infos.empty())
    return false;

  for (const auto &Info : Infos) {
    MachineBasicBlock &MBB = *Info.MBB;
    const SmallVectorImpl<MachineInstr *> &CondBrs = Info.CondBrs;
    MachineInstr *UncondBr = Info.UncondBr;
    LLVM_DEBUG(dbgs() << "spliting block: " << MBB.getName() << "\n");
    if (Info.JumpTableIndex != -1) {
      splitJumpTable(MBB, Info.JumpTableIndex);
      continue;
    }
    // Compute the non-conditional successor as either the target of any
    // unconditional branch or the layout successor.
    MachineBasicBlock *UncondSucc =
        UncondBr ? getBranchDestination(*UncondBr).getMBB()
                 : &*std::next(MachineFunction::iterator(&MBB));

    // Count how many edges there are to any given successor.
    SmallDenseMap<MachineBasicBlock *, int> SuccCounts;
    if (UncondSucc)
      ++SuccCounts[UncondSucc];
    for (auto *CondBr : CondBrs)
      ++SuccCounts[getBranchDestination(*CondBr).getMBB()];

    for (auto *CondBr : CondBrs) {
      MachineBasicBlock &Succ = *getBranchDestination(*CondBr).getMBB();
      int &SuccCount = SuccCounts[&Succ];
      splitEdge(MBB, Succ, SuccCount, CondBr, &UncondBr, nullptr);
      SuccCount--;
    }

    splitEdge(MBB, *UncondSucc, 1, UncondBr, &UncondBr, nullptr);
  }
  return true;
}

void FrameElide::splitEdge(
    MachineBasicBlock &MBB, MachineBasicBlock &Succ, int SuccCount,
    MachineInstr *Br, MachineInstr **UncondBr,
    std::vector<MachineBasicBlock *>::iterator *JumpTableIterator) {
  assert(!Succ.isEHPad() && "Shouldn't get edges to EH pads!");

  MachineFunction &MF = *MBB.getParent();

  MachineBasicBlock &NewMBB = *MF.CreateMachineBasicBlock();

  // We have to insert the new block immediately after the current one as we
  // don't know what layout-successor relationships the successor has and we
  // may not be able to (and generally don't want to) try to fix those up.
  MF.insert(std::next(MachineFunction::iterator(&MBB)), &NewMBB);

  // Update the branch instruction if necessary.
  if (Br) {
    MachineOperand &MO = getBranchDestination(*Br);
    assert(MO.getMBB() == &Succ && "Didn't start with the right target!");
    MO.setMBB(&NewMBB);

    // If this successor was reached through a branch rather than fallthrough,
    // we might have *broken* fallthrough and so need to inject a new
    // unconditional branch.
    if (!*UncondBr) {
      MachineBasicBlock &OldLayoutSucc =
          *std::next(MachineFunction::iterator(&NewMBB));
      assert(MBB.isSuccessor(&OldLayoutSucc) &&
             "Without an unconditional branch, the old layout successor should "
             "be an actual successor!");
      SmallVector<MachineOperand, 4> Cond;
      TII->insertBranch(MBB, &OldLayoutSucc, nullptr, Cond, DebugLoc());
      // Update the unconditional branch now that we've added one.
      *UncondBr = &MBB.back();
    }

    // Insert unconditional "jump Succ" instruction in the new block if
    // necessary.
    if (!NewMBB.isLayoutSuccessor(&Succ)) {
      SmallVector<MachineOperand, 4> Cond;
      TII->insertBranch(NewMBB, &Succ, nullptr, Cond, Br->getDebugLoc());
    }
  } else if (JumpTableIterator) {
    **JumpTableIterator = &NewMBB;
    if (!NewMBB.isLayoutSuccessor(&Succ)) {
      SmallVector<MachineOperand, 4> Cond;
      TII->insertBranch(NewMBB, &Succ, nullptr, Cond, DebugLoc());
    }
  } else {
    assert(!UncondBr &&
           "Cannot have a branchless successor and an unconditional branch!");
    assert(NewMBB.isLayoutSuccessor(&Succ) &&
           "A non-branch successor must have been a layout successor before "
           "and now is a layout successor of the new block.");
  }

  // If this is the only edge to the successor, we can just replace it in the
  // CFG. Otherwise we need to add a new entry in the CFG for the new
  // successor.
  if (SuccCount == 1) {
    MBB.replaceSuccessor(&Succ, &NewMBB);
  } else {
    MBB.splitSuccessor(&Succ, &NewMBB);
  }

  // Hook up the edge from the new basic block to the old successor in the CFG.
  NewMBB.addSuccessor(&Succ);

  // Inherit live-ins from the successor
  for (auto &LI : Succ.liveins())
    NewMBB.addLiveIn(LI);

  LLVM_DEBUG(dbgs() << "  Split edge from '" << MBB.getName() << "' to '"
                    << Succ.getName() << "'.\n");
}

void FrameElide::splitJumpTable(MachineBasicBlock &MBB, int JumpTableIndex) {
  MachineJumpTableInfo *JumpTableInfo = MachineFunc->getJumpTableInfo();
  std::vector<MachineJumpTableEntry> &JumpTables =
      const_cast<std::vector<MachineJumpTableEntry> &>(
          JumpTableInfo->getJumpTables());
  MachineJumpTableEntry &Entry = JumpTables[JumpTableIndex];
  SmallDenseMap<MachineBasicBlock *, int> SuccCounts;
  for (MachineBasicBlock *MBB : Entry.MBBs) {
    ++SuccCounts[MBB];
  }
  for (auto i = Entry.MBBs.begin(), E = Entry.MBBs.end(); i != E; ++i) {
    MachineBasicBlock &Succ = **i;
    if (Succ.pred_size() == 1)
      continue;
    int &SuccCount = SuccCounts[&Succ];
    splitEdge(MBB, Succ, SuccCount, nullptr, nullptr, &i);
    SuccCount--;
  }
}

SmallVector<FrameElide::BlockCondInfo, 16>
FrameElide::collectBlockCondInfo(MachineFunction &MF) {
  SmallVector<BlockCondInfo, 16> Infos;

  // Walk the function and build up a summary for each block's conditions that
  // we need to trace through.
  for (MachineBasicBlock &MBB : MF) {
    // If there are no or only one successor, nothing to do here.
    if (MBB.succ_size() <= 1)
      continue;

    if (MBB.isEHFuncletEntry()) {
      giveUpWithRemarks(ORE, "UnsupportedEHFunclets",
                        "EH Funclets are not supported yet.",
                        MBB.front().getDebugLoc(), &MBB);
      return SmallVector<BlockCondInfo, 16>();
    }

    // We want to reliably handle any conditional branch terminators in the
    // MBB, so we manually analyze the branch. We can handle all of the
    // permutations here, including ones that analyze branch cannot.
    //
    // The approach is to walk backwards across the terminators, resetting at
    // any unconditional non-indirect branch, and track all conditional edges
    // to basic blocks as well as the fallthrough or unconditional successor
    // edge. For each conditional edge, we track the target and the opposite
    // condition code in order to inject a "no-op" cmov into that successor
    // that will harden the predicate. For the fallthrough/unconditional
    // edge, we inject a separate cmov for each conditional branch with
    // matching condition codes. This effectively implements an "and" of the
    // condition flags, even if there isn't a single condition flag that would
    // directly implement that. We don't bother trying to optimize either of
    // these cases because if such an optimization is possible, LLVM should
    // have optimized the conditional *branches* in that way already to reduce
    // instruction count. This late, we simply assume the minimal number of
    // branch instructions is being emitted and use that to guide our cmov
    // insertion.

    BlockCondInfo Info = {&MBB, {}, nullptr, -1};

    // Now walk backwards through the terminators and build up successors they
    // reach and the conditions.
    for (MachineInstr &MI : llvm::reverse(MBB)) {
      // Once we've handled all the terminators, we're done.
      if (!MI.isTerminator())
        break;

      // If we see a non-branch terminator, we can't handle anything so bail.
      if (!MI.isBranch()) {
        Info.CondBrs.clear();
        break;
      }

      // Handle jump table.
      if (MI.isIndirectBranch()) {
        for (MachineBasicBlock::iterator I = MachineBasicBlock::iterator(MI),
                                         E = MBB.begin();
             I != E; I = std::prev(I)) {
          MachineInstr &Prev = *std::prev(I);
          for (unsigned i = 0, E = Prev.getNumOperands(); i < E; ++i) {
            MachineOperand &MO = Prev.getOperand(i);
            if (MO.isJTI()) {
              Info.JumpTableIndex = MO.getIndex();
              break;
            }
          }
        }
        if (Info.JumpTableIndex == -1)
          llvm_unreachable("failed to find jump table index");
      }

      // If we see an unconditional branch, reset our state, clear any
      // fallthrough, and set this is the "else" successor.
      if (MI.isUnconditionalBranch()) {
        Info.CondBrs.clear();
        Info.UncondBr = &MI;
        continue;
      }

      // We have a vanilla conditional branch, add it to our list.
      Info.CondBrs.push_back(&MI);
    }

    if (Info.CondBrs.empty() && Info.JumpTableIndex == -1) {
      LLVM_DEBUG(dbgs() << "WARNING: unable to split successors of block:\n";
                 MBB.dump());
      continue;
    }

    Infos.push_back(Info);
  }

  return Infos;
}

MachineOperand &FrameElide::getBranchDestination(MachineInstr &MI) {
  for (unsigned i = 0, E = MI.getNumOperands(); i < E; ++i) {
    MachineOperand &MO = MI.getOperand(i);
    if (MO.isMBB())
      return MO;
  }
  llvm_unreachable("could not find mbb");
}

void FrameElide::markBlocks() {
  for (MachineBasicBlock &MBB : *MachineFunc) {
    MBB.setMustNotInFrame(true);
  }

  for (MachineBasicBlock &MBB : *MachineFunc) {
    LLVM_DEBUG(dbgs() << "Look into: " << MBB.getNumber() << ' '
                      << MBB.getName() << '\n');

    if (!MBB.mustNotInFrame())
      continue;
    for (const MachineInstr &MI : MBB) {
      if (!useOrDefCSROrFI(MI))
        continue;
      MBB.setMustNotInFrame(false);
      break;
    }
  }
}

void FrameElide::propagateMarks() {
  while (propagateInOrder() || propagateReversed()) {
  }
}

bool FrameElide::propagateInOrder() {
  bool Changed = false;
  for (MachineBasicBlock &MBB : *MachineFunc) {
    Changed |= propagateIntoBlock(MBB);
  }
  return Changed;
}

bool FrameElide::propagateReversed() {
  bool Changed = false;
  for (MachineBasicBlock &MBB :
       make_range(MachineFunc->rbegin(), MachineFunc->rend())) {
    Changed |= propagateIntoBlock(MBB);
  }
  return Changed;
}

bool FrameElide::propagateIntoBlock(MachineBasicBlock &MBB) {
  // Already marked, nothing to do...
  if (!MBB.mustNotInFrame())
    return false;

  // Propagate towards the end ("downwards") if there is a predecessor needing
  // a frame, but don't "bleed" from deferred code to non-deferred code.
  auto ShouldBeInFramePred = [&](MachineBasicBlock *Pred) {
    if (Pred->mustNotInFrame())
      return false;
    if (!deferredBlock(*Pred))
      return true;
    // Stop bleeding
    return deferredBlock(MBB);
  };
  for (MachineBasicBlock *Pred : MBB.predecessors()) {
    if (ShouldBeInFramePred(Pred)) {
      MBB.setMustNotInFrame(false);
      return true;
    }
  }

  // Propagate towards start ("upwards")
  bool NeedFrameSuccessors = false;
  if (MBB.succ_size() == 1) {
    // For single successors, propagate the needs_frame information.
    NeedFrameSuccessors = !(*MBB.succ_begin())->mustNotInFrame();
  } else {
    for (MachineBasicBlock *Succ : MBB.successors()) {
      assert(Succ->pred_size() == 1);
      if (!deferredBlock(*Succ)) {
        if (!Succ->mustNotInFrame()) {
          NeedFrameSuccessors = true;
        } else {
          return false;
        }
      }
    }
  }
  if (NeedFrameSuccessors) {
    MBB.setMustNotInFrame(false);
    return true;
  } else {
    return false;
  }
}

bool FrameElide::deferredBlock(MachineBasicBlock &MBB) {
  return DeferredBlocks.test(MBB.getNumber());
}

void FrameElide::determineDeferredBlocks() {
  DeferredBlocks.clear();
  DeferredBlocks.resize(MachineFunc->getNumBlockIDs(), true);
  DenseSet<unsigned> Visited;
  SmallVector<unsigned, 4> WorkingList;

  WorkingList.push_back(0);

  while (!WorkingList.empty()) {
    unsigned ID = WorkingList.back();
    WorkingList.pop_back();
    auto insert_result = Visited.insert(ID);
    if (!insert_result.second)
      continue;
    DeferredBlocks.reset(ID);
    MachineBasicBlock *MBB = MachineFunc->getBlockNumbered(ID);
    for (auto I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I) {
      const BranchProbability BP = MBPI->getEdgeProbability(MBB, *I);
      double Prob =
          static_cast<double>(BP.getNumerator()) / BP.getDenominator();

      if (Prob > 0.05) {
        WorkingList.push_back((*I)->getNumber());
      }
    }
  }
  for (unsigned ID = 0; ID < DeferredBlocks.size(); ++ID) {
    if (DeferredBlocks[ID]) {
      MachineBasicBlock *MBB = MachineFunc->getBlockNumbered(ID);
    }
  }
}
