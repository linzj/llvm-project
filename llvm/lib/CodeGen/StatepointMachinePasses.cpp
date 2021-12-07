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
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveStacks.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegisterUsageInfo.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/IR/Module.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <list>
#include <map>
#include <string>
#include <unordered_set>

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
  const DenseSet<Register> getVirtRegHasPhysSet();

  DenseSet<Register> VirtRegHasPhysSet;
  SlotIndexes *Indexes;
  LiveIntervals *LIS;
  LiveStacks *LSS;
  VirtRegMap *VRM;
  MachineRegisterInfo *MRI;
};

static MachineInstr *getSingleDef(const MachineRegisterInfo &MRI,
                                  unsigned Reg) {
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
  SmallPtrSet<MachineInstr *, 8> RemoveSet;

  auto IsValidCopyMI = [&](const MachineInstr &MI) {
    if (MI.getOpcode() == TargetOpcode::RELOCATE_DEF)
      return true;
    if (!MI.isFullCopy())
      return false;
    Register SrcReg = MI.getOperand(1).getReg();
    if (!Register::isVirtualRegister(SrcReg))
      return false;
    Register DstReg = MI.getOperand(0).getReg();
    if (!Register::isVirtualRegister(DstReg))
      return false;
    // ARM use copy to cast from FP to SI. Check it.
    if (MRI->getRegClass(DstReg) != MRI->getRegClass(SrcReg))
      return false;
    return true;
  };

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      if (IsValidCopyMI(MI)) {
        // find the real source.
        Register Src = MI.getOperand(1).getReg();
        MachineInstr *DefOfSrc = getSingleDef(*MRI, Src);
        while (DefOfSrc) {
          if (!IsValidCopyMI(*DefOfSrc))
            break;
          Src = DefOfSrc->getOperand(1).getReg();
          DefOfSrc = getSingleDef(*MRI, Src);
        }
        // Replace Dst with Src.
        Register Dst = MI.getOperand(0).getReg();
        MRI->replaceRegWith(Dst, Src);
        RemoveSet.insert(&MI);
      }
    }
  }
  if (RemoveSet.empty())
    return false;

  for (auto MI : RemoveSet) {
    MI->removeFromParent();
  }
  return !RemoveSet.empty();
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
  unsigned StartIdx = 0;
  uint64_t ID = 0;
  switch (StatePoint->getOpcode()) {
  case TargetOpcode::STATEPOINT: {
    StatepointOpers Op(StatePoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    int64_t NumDeoptArgs = StatePoint->getOperand(StartIdx + 5).getImm();
    StartIdx += 6 + NumDeoptArgs;
    break;
  }
  default:
    llvm_unreachable("unexpected stackmap opcode");
  }

  if (StartIdx == StatePoint->getNumOperands())
    return false;

  bool Changed = false;
  for (unsigned i = StatePoint->getNumOperands() - 1; i >= StartIdx; --i) {
    MachineOperand &MO = StatePoint->getOperand(i);
    if (!MO.isReg() || Register::isPhysicalRegister(MO.getReg())) {
      continue;
    }
    MRI->addStatepointObserved(MO.getReg());
    StatePoint->RemoveOperand(i);
    Changed = true;
  }

  return Changed;
}

#undef DEBUG_TYPE
#define DEBUG_TYPE "statepoint-rewrite"
char StatepointRewrite::ID = 0;

INITIALIZE_PASS_BEGIN(StatepointRewrite, DEBUG_TYPE,
                      "Rewrite Statepoints PostRA", false, false)
INITIALIZE_PASS_DEPENDENCY(SlotIndexes)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_DEPENDENCY(LiveStacks)
INITIALIZE_PASS_DEPENDENCY(VirtRegMap)
INITIALIZE_PASS_END(StatepointRewrite, DEBUG_TYPE, "Rewrite Statepoints PostRA",
                    false, false)

StatepointRewrite::StatepointRewrite()
    : MachineFunctionPass(ID), Indexes(nullptr), LSS(nullptr), VRM(nullptr) {
  PassRegistry &Registry = *PassRegistry::getPassRegistry();
  initializeStatepointRewritePass(Registry);
}

void StatepointRewrite::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<SlotIndexes>();
  AU.addRequired<LiveIntervals>();
  AU.addRequired<LiveStacks>();
  AU.addRequired<VirtRegMap>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

bool StatepointRewrite::runOnMachineFunction(MachineFunction &MF) {
  Indexes = &getAnalysis<SlotIndexes>();
  LIS = &getAnalysis<LiveIntervals>();
  LSS = &getAnalysis<LiveStacks>();
  VRM = &getAnalysis<VirtRegMap>();
  MRI = &MF.getRegInfo();
  VirtRegHasPhysSet.clear();
  return rewriteStatepoints(MF);
}

bool StatepointRewrite::rewriteStatepoints(MachineFunction &MF) {
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
    Changed |= rewriteStatepoint(MF, MI);
  return Changed;
}

bool StatepointRewrite::rewriteStatepoint(MachineFunction &MF,
                                          MachineInstr *StatePoint) {
  unsigned StartIdx = 0;
  uint64_t ID = 0;
  switch (StatePoint->getOpcode()) {
  case TargetOpcode::STATEPOINT: {
    StatepointOpers Op(StatePoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  default:
    llvm_unreachable("unexpected stackmap opcode");
  }
  MachineRegisterInfo *MRI = &MF.getRegInfo();
  MachineFrameInfo *MFI = &MF.getFrameInfo();
  // Ready to rebuild the StatePoint.
  const TargetSubtargetInfo &STI = MF.getSubtarget();
  const TargetInstrInfo *TII = STI.getInstrInfo();
  MachineInstr *NewMI = MF.CreateMachineInstr(TII->get(StatePoint->getOpcode()),
                                              StatePoint->getDebugLoc(), true);
  MachineInstrBuilder MIB(MF, NewMI);
  for (unsigned i = 0, E = StatePoint->getNumOperands(); i < E; ++i)
    MIB.add(StatePoint->getOperand(i));

  // Check Any Stack Slot overlaps this StatePoint.
  SlotIndex Index = Indexes->getInstructionIndex(*StatePoint);
  Index = Index.getNextSlot();

  auto AddStackSlot = [&](int StackSlot) {
    LiveInterval &Interval = LSS->getInterval(StackSlot);
    if (!Interval.liveAt(Index))
      return;
    // Add the Stack Slot Info.
    int SpillSize = MFI->getObjectSize(StackSlot);
    MIB.addImm(StackMaps::IndirectMemRefOp);
    MIB.addImm(SpillSize);
    MIB.addFrameIndex(StackSlot);
    MIB.addImm(0);
  };

  auto AddPhysIfLiveOut = [&](Register Reg) {
    if (!LIS->hasInterval(Reg) || !LIS->getInterval(Reg).liveAt(Index)) {
      return false;
    }
    // Must mark renamable or Machine Copy Propagation will not change it.
    MIB.addReg(VRM->getPhys(Reg), RegState::Renamable);
    return true;
  };

  // Add Stack Slots
  for (int i = 0, e = MFI->getObjectIndexEnd(); i != e; ++i) {
    if (MFI->isStatepointSpillSlotObjectIndex(i))
      AddStackSlot(i);
  }

  for (Register r : getVirtRegHasPhysSet()) {
    AddPhysIfLiveOut(r);
  }

  MachineBasicBlock *MBB = StatePoint->getParent();
  MachineBasicBlock::iterator Pos = StatePoint;
  MBB->insert(Pos, NewMI);
  LIS->ReplaceMachineInstrInMaps(*StatePoint, *NewMI);
  MBB->erase(StatePoint);
  return true;
}

const DenseSet<Register> StatepointRewrite::getVirtRegHasPhysSet() {
  if (!VirtRegHasPhysSet.empty())
    return VirtRegHasPhysSet;

  for (unsigned i = 0, E = MRI->getNumVirtRegs(); i != E; ++i) {
    Register r(Register::index2VirtReg(i));
    if (!VRM->hasPhys(r))
      continue;
    if (!MRI->isStatepointObserved(r) &&
        !MRI->isStatepointObserved(VRM->getOriginal(r)))
      continue;
    VirtRegHasPhysSet.insert(r);
  }
  return VirtRegHasPhysSet;
}

FunctionPass *llvm::createStatepointSimplifyPass() {
  return new StatepointSimplify();
}

FunctionPass *llvm::createStatepointRewritePass() {
  return new StatepointRewrite();
}
