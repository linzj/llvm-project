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
  bool removeVarFromPatchpoints(MachineFunction &MF);

  bool removeVarFromPatchpoint(MachineFunction &MF, MachineInstr *MI);

  void replaceDstWithSrc(unsigned Dst, unsigned Src);

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
  bool rewritePatchpoints(MachineFunction &);
  bool rewritePatchpoint(MachineFunction &, MachineInstr *MI);
  SlotIndexes *Indexes;
  LiveIntervals *LIS;
  LiveStacks *LSS;
  VirtRegMap *VRM;
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
  Changed |= removeVarFromPatchpoints(MF);
  return Changed;
}

bool StatepointSimplify::foldRelocateDef(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  const TargetSubtargetInfo &STI = MF.getSubtarget();
  TII = STI.getInstrInfo();
  std::unordered_set<MachineInstr *> RemoveSet;
  std::unordered_set<MachineInstr *> PHIs;
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      if (MI.getOpcode() == TargetOpcode::RELOCATE_DEF) {
        // find the real source.
        Register Src = MI.getOperand(1).getReg();
        MachineInstr *DefOfSrc = getSingleDef(*MRI, Src);
        while (DefOfSrc) {
          unsigned MaybeSrc = 0;
          switch (DefOfSrc->getOpcode()) {
          case TargetOpcode::RELOCATE_DEF:
          case TargetOpcode::COPY:
            MaybeSrc = DefOfSrc->getOperand(1).getReg();
            break;
          default:
            break;
          }
          if (!MaybeSrc)
            break;
          if (!Register::isVirtualRegister(MaybeSrc))
            break;
          Src = MaybeSrc;
          DefOfSrc = getSingleDef(*MRI, Src);
        }
        // Replace Dst with Src.
        Register Dst = MI.getOperand(0).getReg();
        replaceDstWithSrc(Dst, Src);
        RemoveSet.emplace(&MI);
      } else if (MI.isPHI()) {
        PHIs.emplace(&MI);
      }
    }
  }
  if (RemoveSet.empty())
    return false;
  // Remove PHIs
  bool Changed;
  do {
    Changed = false;
    for (auto It = PHIs.begin(); It != PHIs.end();) {
      auto MPhi = *It;
      // Preprocess phi. So that all the copy defined operands will be replaced
      // to their sources.
      for (unsigned I = 1, E = MPhi->getNumOperands(); I < E; I += 2) {
        MachineOperand &MO = MPhi->getOperand(I);
        Register Reg = MO.getReg();
        MachineInstr *MaybeCopy = getSingleDef(*MRI, Reg);
        if (!MaybeCopy)
          continue;
        if (!MaybeCopy->isFullCopy())
          continue;
        Register Src = MaybeCopy->getOperand(1).getReg();
        if (!Register::isVirtualRegister(Src))
          continue;
        Register Dst = MaybeCopy->getOperand(0).getReg();
        LLVM_DEBUG(dbgs() << "foldRelocateDef: Replacing Copy " << *MaybeCopy
                          << " with " << printReg(Src) << " for " << *MPhi);
        replaceDstWithSrc(Dst, Src);
        RemoveSet.emplace(MaybeCopy);
        Changed = true;
      }
      Register FirstReg;
      Register MPhiDef = MPhi->getOperand(0).getReg();
      bool MPhiOneDef = MRI->hasOneDef(MPhiDef);
      for (unsigned I = 1, E = MPhi->getNumOperands(); I < E; I += 2) {
        Register MaybeFirstReg = MPhi->getOperand(I).getReg();
        if (MaybeFirstReg != MPhiDef) {
          FirstReg = MaybeFirstReg;
          break;
        }
      }
      bool NotEqualToFirstReg = false;
      for (unsigned I = 3, E = MPhi->getNumOperands(); I < E; I += 2) {
        const MachineOperand &MO = MPhi->getOperand(I);
        if (FirstReg != MO.getReg() &&
            (MO.getReg() != MPhiDef || !MPhiOneDef)) {
          NotEqualToFirstReg = true;
          break;
        }
      }
      if (NotEqualToFirstReg) {
        ++It;
        continue;
      }
      LLVM_DEBUG(dbgs() << "foldRelocateDef: Replacing Phi" << *MPhi << " with "
                        << printReg(FirstReg) << "\n");
      replaceDstWithSrc(MPhi->getOperand(0).getReg(), FirstReg);
      RemoveSet.emplace(MPhi);
      Changed = true;
      PHIs.erase(It++);
    }
  } while (Changed);
  for (auto MI : RemoveSet) {
    MI->removeFromParent();
  }
  return !RemoveSet.empty();
}

bool StatepointSimplify::removeVarFromPatchpoints(MachineFunction &MF) {
  bool Changed = false;
  SmallVector<MachineInstr *, 8> WorkList;
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      switch (MI.getOpcode()) {
      case TargetOpcode::STACKMAP:
      case TargetOpcode::PATCHPOINT:
      case TargetOpcode::TCPATCHPOINT:
      case TargetOpcode::STATEPOINT:
        WorkList.emplace_back(&MI);
        break;
      }
    }
  }
  for (MachineInstr *MI : WorkList)
    Changed |= removeVarFromPatchpoint(MF, MI);
  return Changed;
}

bool StatepointSimplify::removeVarFromPatchpoint(MachineFunction &MF,
                                                 MachineInstr *PatchPoint) {
  unsigned StartIdx = 0;
  uint64_t ID = 0;
  switch (PatchPoint->getOpcode()) {
  case TargetOpcode::STACKMAP: {
    StackMapOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  case TargetOpcode::TCPATCHPOINT:
  case TargetOpcode::PATCHPOINT: {
    PatchPointOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  case TargetOpcode::STATEPOINT: {
    StatepointOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  default:
    llvm_unreachable("unexpected stackmap opcode");
  }

  if (StartIdx == PatchPoint->getNumOperands())
    return false;

  MachineInstr *NewMI = MF.CreateMachineInstr(TII->get(PatchPoint->getOpcode()),
                                              PatchPoint->getDebugLoc(), true);
  MachineInstrBuilder MIB(MF, NewMI);

  // No need to fold return, the meta data, and function arguments
  for (unsigned i = 0; i < StartIdx; ++i)
    MIB.add(PatchPoint->getOperand(i));

  auto &IDMap = MRI->getPatchpointIDMap();
  for (unsigned i = StartIdx; i < PatchPoint->getNumOperands(); ++i) {
    MachineOperand &MO = PatchPoint->getOperand(i);
    if (!MO.isReg() || Register::isPhysicalRegister(MO.getReg())) {
      MIB.add(MO);
      continue;
    }
    Register Reg = MO.getReg();
    unsigned SpillSize;
    unsigned SpillOffset;
    // Compute the spill slot size and offset.
    const TargetRegisterClass *RC = MRI->getRegClass(Reg);
    bool Valid =
        TII->getStackSlotRange(RC, MO.getSubReg(), SpillSize, SpillOffset, MF);
    if (!Valid)
      report_fatal_error("cannot spill patchpoint subregister operand");
    IDMap[ID].emplace_back(
        MachineRegisterInfo::PatchpointRegInfo{Reg, SpillSize, SpillOffset});
  }
  MachineBasicBlock *MBB = PatchPoint->getParent();
  MachineBasicBlock::iterator Pos = PatchPoint;
  MBB->insert(Pos, NewMI);
  MBB->erase(PatchPoint);
  return !IDMap.empty();
}

void StatepointSimplify::replaceDstWithSrc(unsigned Dst, unsigned Src) {
  LLVM_DEBUG(dbgs() << "Replace the use of " << printReg(Dst) << " with "
                    << printReg(Src) << "\n");
  for (auto it = MRI->use_begin(Dst); it != MRI->use_end();) {
    auto &MO = *it;
    ++it;
    LLVM_DEBUG(dbgs() << "setReg for " << MO << " in MI " << *MO.getParent()
                      << '\n');
    MO.setReg(Src);
  }
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
  return rewritePatchpoints(MF);
}

bool StatepointRewrite::rewritePatchpoints(MachineFunction &MF) {
  bool Changed = false;
  SmallVector<MachineInstr *, 8> WorkList;
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    for (MachineInstr &MI : *MBB) {
      switch (MI.getOpcode()) {
      case TargetOpcode::STACKMAP:
      case TargetOpcode::PATCHPOINT:
      case TargetOpcode::TCPATCHPOINT:
      case TargetOpcode::STATEPOINT:
        WorkList.emplace_back(&MI);
        break;
      }
    }
  }
  for (MachineInstr *MI : WorkList)
    Changed |= rewritePatchpoint(MF, MI);
  return Changed;
}

bool StatepointRewrite::rewritePatchpoint(MachineFunction &MF,
                                          MachineInstr *PatchPoint) {
  unsigned StartIdx = 0;
  uint64_t ID = 0;
  switch (PatchPoint->getOpcode()) {
  case TargetOpcode::STACKMAP: {
    StackMapOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  case TargetOpcode::TCPATCHPOINT:
  case TargetOpcode::PATCHPOINT: {
    PatchPointOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  case TargetOpcode::STATEPOINT: {
    StatepointOpers Op(PatchPoint);
    StartIdx = Op.getVarIdx();
    ID = Op.getID();
    break;
  }
  default:
    llvm_unreachable("unexpected stackmap opcode");
  }
  MachineRegisterInfo *MRI = &MF.getRegInfo();
  const auto &IDMap = MRI->getPatchpointIDMap();
  if (IDMap.empty())
    return false;
  auto Found = IDMap.find(ID);
  if (Found == IDMap.end())
    return false;
  // Ready to rebuild the patchpoint.
  const TargetSubtargetInfo &STI = MF.getSubtarget();
  const TargetInstrInfo *TII = STI.getInstrInfo();
  MachineInstr *NewMI = MF.CreateMachineInstr(TII->get(PatchPoint->getOpcode()),
                                              PatchPoint->getDebugLoc(), true);
  MachineInstrBuilder MIB(MF, NewMI);
  for (unsigned i = 0, E = PatchPoint->getNumOperands(); i < E; ++i)
    MIB.add(PatchPoint->getOperand(i));

  // Check Any Stack Slot overlaps this PatchPoint.
  SlotIndex Index = Indexes->getInstructionIndex(*PatchPoint);
  const auto &RegInfoVector = Found->second;
  auto AddStackSlot =
      [&](const MachineRegisterInfo::PatchpointRegInfo &RegInfo) {
        int StackSlot = VRM->getStackSlot(RegInfo.Reg);
        if (StackSlot == VirtRegMap::NO_STACK_SLOT)
          return;
        LiveInterval &Interval = LSS->getInterval(StackSlot);
        if (!Interval.liveAt(Index))
          return;
        // Add the Stack Slot Info.
        MIB.addImm(StackMaps::IndirectMemRefOp);
        MIB.addImm(RegInfo.SpillSize);
        MIB.addFrameIndex(StackSlot);
        MIB.addImm(RegInfo.SpillOffset);
      };

  auto AddPhysIfLiveOut = [&](Register Reg) {
    if (!VRM->hasPhys(Reg)) {
      return false;
    }
    if (!LIS->hasInterval(Reg) || !LIS->getInterval(Reg).liveAt(Index)) {
      return false;
    }
    MIB.addReg(VRM->getPhys(Reg));
    return true;
  };

  for (const auto &RegInfo : RegInfoVector) {
    LLVM_DEBUG(
        dbgs() << "Trying to rewrite for ID: " << ID << "; Reg: "
               << printReg(RegInfo.Reg, MF.getRegInfo().getTargetRegisterInfo())
               << "\n");
    if (Register::isPhysicalRegister(RegInfo.Reg)) {
      MIB.addReg(RegInfo.Reg);
      continue;
    }
    assert(VRM->getOriginal(RegInfo.Reg) == RegInfo.Reg);
    AddStackSlot(RegInfo);
    // Add live phys registers to MIB if targeted.
    if (AddPhysIfLiveOut(RegInfo.Reg))
      continue;
    const auto &split = VRM->collectSplitRegs(RegInfo.Reg);
    for (auto Reg : split) {
      if (AddPhysIfLiveOut(Reg))
        break;
    }
  }
  MachineBasicBlock *MBB = PatchPoint->getParent();
  MachineBasicBlock::iterator Pos = PatchPoint;
  MBB->insert(Pos, NewMI);
  LIS->ReplaceMachineInstrInMaps(*PatchPoint, *NewMI);
  MBB->erase(PatchPoint);
  return true;
}

FunctionPass *llvm::createStatepointSimplifyPass() {
  return new StatepointSimplify();
}

FunctionPass *llvm::createStatepointRewritePass() {
  return new StatepointRewrite();
}
