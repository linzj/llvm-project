//===- SplitStatepointBlock.cpp - Split the block use Statepoint ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// The pass split the block contains state point calls. It will split after the
// state point call sequence.
//
//===----------------------------------------------------------------------===//

#include "llvm/ADT/APInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Use.h"
#include "llvm/IR/User.h"
#include "llvm/IR/Value.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetIntrinsicInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/Local.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <string>
#include <utility>

#define DEBUG_TYPE "split-statepoint-block"

namespace llvm {

namespace {

class SplitStatepointBlockLegacyPass : public FunctionPass {
  SmallVector<BasicBlock *, 16> WorkList;

  void handleStatepoint(BasicBlock *BB, CallInst *Call) {
    BasicBlock *New = BB->splitBasicBlock(++(Call->getIterator()),
                                          BB->getName() + ".statepoint.split");
    WorkList.push_back(New);
  }

public:
  static char ID; // Pass identification, replacement for typeid..

  SplitStatepointBlockLegacyPass() : FunctionPass(ID) {
    initializeSplitStatepointBlockLegacyPassPass(
        *PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<TargetPassConfig>();
  }

  bool runOnFunction(Function &F) override {
    const TargetMachine *TM = nullptr;
    TM = &getAnalysis<TargetPassConfig>().getTM<TargetMachine>();
    bool IsTargetingDart =
        TM->getTargetTriple().getEnvironment() == Triple::Dart;
    if (!IsTargetingDart)
      return false;
    for (BasicBlock &BB : F) {
      WorkList.push_back(&BB);
    }
    bool Changed = false;
    while (!WorkList.empty()) {
      BasicBlock *BB = WorkList.pop_back_val();
      for (Instruction &I : *BB) {
        CallInst *Call = dyn_cast<CallInst>(&I);
        if (Call == nullptr) continue;
        if (Function *F = Call->getCalledFunction()) {
          if (F->isDeclaration()) {
            // Is this an LLVM intrinsic or a target-specific intrinsic?
            unsigned IID = F->getIntrinsicID();

            if (IID == Intrinsic::experimental_gc_statepoint) {
              handleStatepoint(BB, Call);
              Changed = true;
            }
          }
        }
      }
    }
    return Changed;
  }
};

} // end anonymous namespace
} // namespace llvm

using namespace llvm;
char SplitStatepointBlockLegacyPass::ID = 0;

INITIALIZE_PASS_BEGIN(SplitStatepointBlockLegacyPass, DEBUG_TYPE,
                      "Split Statepoint Block pass", false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_END(SplitStatepointBlockLegacyPass, DEBUG_TYPE,
                    "Split Statepoint Block pass", false, false)

FunctionPass *llvm::createSplitStatepointBlockPass() {
  return new SplitStatepointBlockLegacyPass();
}
