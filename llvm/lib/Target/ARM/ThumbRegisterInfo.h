//===- ThumbRegisterInfo.h - Thumb Register Information Impl -*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Thumb implementation of the TargetRegisterInfo
// class. With the exception of emitLoadConstPool Thumb2 tracks
// ARMBaseRegisterInfo, Thumb1 overloads the functions below.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_ARM_THUMB1REGISTERINFO_H
#define LLVM_LIB_TARGET_ARM_THUMB1REGISTERINFO_H

#include "ARMBaseRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

namespace llvm {
  class ARMSubtarget;
  class ARMBaseInstrInfo;

struct ThumbRegisterInfo : public ARMBaseRegisterInfo {
public:
  explicit ThumbRegisterInfo(const Triple &);

  const TargetRegisterClass *
  getLargestLegalSuperClass(const TargetRegisterClass *RC,
                            const MachineFunction &MF) const override;

  const TargetRegisterClass *
  getPointerRegClass(const MachineFunction &MF,
                     unsigned Kind = 0) const override;

  /// emitLoadConstPool - Emits a load from constpool to materialize the
  /// specified immediate.
  void
  emitLoadConstPool(MachineBasicBlock &MBB, MachineBasicBlock::iterator &MBBI,
                    const DebugLoc &dl, unsigned DestReg, unsigned SubIdx,
                    int Val, ARMCC::CondCodes Pred = ARMCC::AL,
                    unsigned PredReg = 0,
                    unsigned MIFlags = MachineInstr::NoFlags) const override;

  // rewrite MI to access 'Offset' bytes from the FP. Update Offset to be
  // however much remains to be handled. Return 'true' if no further
  // work is required.
  bool rewriteFrameIndex(MachineBasicBlock::iterator II, unsigned FrameRegIdx,
                         unsigned FrameReg, int &Offset,
                         const ARMBaseInstrInfo &TII) const;
  void resolveFrameIndex(MachineInstr &MI, unsigned BaseReg,
                         int64_t Offset) const override;
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;
  bool useFPForScavengingIndex(const MachineFunction &MF) const override;
};
}

#endif
