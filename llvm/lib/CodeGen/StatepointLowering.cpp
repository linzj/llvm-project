//===- StatepointLowering.cpp - Lower Statepoint related instrs -------===//
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
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegisterUsageInfo.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/Module.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <list>
#include <map>
#include <string>
#include <unordered_set>

#define DEBUG_TYPE "statepoint-lowering"
using namespace llvm;
namespace {

class StatepointLowering : public MachineFunctionPass {
public:
  StatepointLowering();

  StringRef getPassName() const override { return "Statepoint Lowering"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  static char ID;

private:
  bool foldRelocateDef(MachineFunction &F);
  /// Remove the redundant move immediate from statepoints.
  bool removeMoveImmediateFromPatchpoint(MachineFunction &MF);

  bool removeMoveImmediateFromPatchpoint(MachineFunction &MF, MachineInstr *MI);

  void replaceDstWithSrc(unsigned Dst, unsigned Src);

  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;
};

static MachineInstr *getSingleDef(const MachineRegisterInfo &MRI,
                                  unsigned Reg) {
  if (!MRI.hasOneDef(Reg))
    return nullptr;
  auto defs_iterator = MRI.def_begin(Reg);
  return defs_iterator->getParent();
}

static bool shouldFoldAsConstant(const MachineOperand &MO,
                                 const MachineRegisterInfo &MRI, int64_t *imm) {
  if (!MO.isReg())
    return false;
  const MachineOperand *Target = &MO;
  std::unordered_set<unsigned> Visited;
  while (true) {
    auto InsertResult = Visited.emplace(Target->getReg());
    // Avoid loop.
    if (!InsertResult.second)
      return false;
    const MachineInstr *MI = getSingleDef(MRI, Target->getReg());
    if (!MI)
      return false;
    if (MI->isFullCopy() &&
        Register::isVirtualRegister(MI->getOperand(1).getReg())) {
      Target = &MI->getOperand(1);
      continue;
    }
    if (!MI->isMoveImmediate())
      return false;
    const MachineOperand &ImmOperand = MI->getOperand(1);
    if (!ImmOperand.isImm())
      return false;
    *imm = ImmOperand.getImm();
    return true;
  }
}

} // end of anonymous namespace

INITIALIZE_PASS(StatepointLowering, "statepoint-lowering",
                "Statepoint lowering", false, false)

char StatepointLowering::ID = 0;

StatepointLowering::StatepointLowering()
    : MachineFunctionPass(ID), MRI(nullptr), TII(nullptr) {
  PassRegistry &Registry = *PassRegistry::getPassRegistry();
  initializeStatepointLoweringPass(Registry);
}

bool StatepointLowering::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = foldRelocateDef(MF);
  Changed |= removeMoveImmediateFromPatchpoint(MF);
  return Changed;
}

bool StatepointLowering::foldRelocateDef(MachineFunction &MF) {
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

bool StatepointLowering::removeMoveImmediateFromPatchpoint(
    MachineFunction &MF) {
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
    Changed |= removeMoveImmediateFromPatchpoint(MF, MI);
  return Changed;
}

bool StatepointLowering::removeMoveImmediateFromPatchpoint(
    MachineFunction &MF, MachineInstr *PatchPoint) {
  unsigned StartIdx = 0;
  bool Changed = false;
  switch (PatchPoint->getOpcode()) {
  case TargetOpcode::STACKMAP: {
    StartIdx = StackMapOpers(PatchPoint).getVarIdx();
    break;
  }
  case TargetOpcode::TCPATCHPOINT:
  case TargetOpcode::PATCHPOINT: {
    StartIdx = PatchPointOpers(PatchPoint).getVarIdx();
    break;
  }
  case TargetOpcode::STATEPOINT: {
    StartIdx = StatepointOpers(PatchPoint).getVarIdx();
    break;
  }
  default:
    llvm_unreachable("unexpected stackmap opcode");
  }

  MachineInstr *NewMI = MF.CreateMachineInstr(TII->get(PatchPoint->getOpcode()),
                                              PatchPoint->getDebugLoc(), true);
  MachineInstrBuilder MIB(MF, NewMI);

  // No need to fold return, the meta data, and function arguments
  for (unsigned i = 0; i < StartIdx; ++i)
    MIB.add(PatchPoint->getOperand(i));
  for (unsigned i = StartIdx; i < PatchPoint->getNumOperands(); ++i) {
    MachineOperand &MO = PatchPoint->getOperand(i);
    int64_t Imm;
    if (!shouldFoldAsConstant(MO, *MRI, &Imm)) {
      MIB.add(MO);
      continue;
    }
    LLVM_DEBUG(dbgs() << "removeMoveImmediateFromPatchpoint: Removing " << MO
                      << " from " << *PatchPoint);
    MIB.addImm(StackMaps::ConstantOp);
    MIB.addImm(Imm);
    Changed |= true;
  }
  MachineBasicBlock *MBB = PatchPoint->getParent();
  MachineBasicBlock::iterator Pos = PatchPoint;
  MBB->insert(Pos, NewMI);
  MBB->erase(PatchPoint);
  return Changed;
}

void StatepointLowering::replaceDstWithSrc(unsigned Dst, unsigned Src) {
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

FunctionPass *llvm::createStatepointLoweringPass() {
  return new StatepointLowering();
}
