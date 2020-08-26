; RUN: llc < %s | FileCheck %s
; ModuleID = 'test-no-split.cc'
source_filename = "test-no-split.cc"
target datalayout = "e-m:e-i8:8:32-i16:16:32-i64:64-i128:128-n32:64-S128"
target triple = "aarch64-unknown-linux-android"

%struct.Object = type <{ i8, i16, i8, i32, [0 x %struct.Object*] }>

; Function Attrs: nounwind
define hidden void @_Z3Fooh(i8 %barrier_mask) local_unnamed_addr #0 {
; CHECK-LABEL: .LBB0_2:
; CHECK: ldr x{{[0-9]+}}, [sp, #{{[0-9]+}}]
; CHECK-NEXT: mov x{{[0-9]+}}, x{{[0-9]+}}
; CHECK-LABEL: .LBB0_14:
; CHECK-NEXT: mov x{{[0-9]+}}, x{{[0-9]+}}
entry:
  %0 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %1 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %arrayidx.i = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 4, i64 0
  store %struct.Object* %1, %struct.Object** %arrayidx.i, align 1, !tbaa !5
  %2 = ptrtoint %struct.Object* %1 to i64
  %and.i = and i64 %2, 1
  %cmp.i = icmp eq i64 %and.i, 0
  br i1 %cmp.i, label %_ZL16AllocateAndStoreP6Objectih.exit, label %if.end.i

if.end.i:                                         ; preds = %entry
  %barrier.i = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 0
  %3 = load i8, i8* %barrier.i, align 1, !tbaa !9
  %4 = lshr i8 %3, 3
  %barrier1.i = getelementptr inbounds %struct.Object, %struct.Object* %1, i64 0, i32 0
  %5 = load i8, i8* %barrier1.i, align 1, !tbaa !9
  %or20.i = or i8 %4, %5
  %and421.i = and i8 %or20.i, %barrier_mask
  %cmp5.i = icmp eq i8 %and421.i, 0
  br i1 %cmp5.i, label %if.then7.i, label %_ZL16AllocateAndStoreP6Objectih.exit, !prof !10

if.then7.i:                                       ; preds = %if.end.i
  tail call void asm sideeffect "", "{x0},{x0},{x26},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"(%struct.Object* nonnull %0, %struct.Object* nonnull %0, %struct.Object** nonnull %arrayidx.i) #1, !srcloc !11
  br label %_ZL16AllocateAndStoreP6Objectih.exit

_ZL16AllocateAndStoreP6Objectih.exit:             ; preds = %entry, %if.end.i, %if.then7.i
  %6 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %arrayidx.i10 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 4, i64 1
  store %struct.Object* %6, %struct.Object** %arrayidx.i10, align 1, !tbaa !5
  %7 = ptrtoint %struct.Object* %6 to i64
  %and.i11 = and i64 %7, 1
  %cmp.i12 = icmp eq i64 %and.i11, 0
  br i1 %cmp.i12, label %_ZL16AllocateAndStoreP6Objectih.exit20, label %if.end.i18

if.end.i18:                                       ; preds = %_ZL16AllocateAndStoreP6Objectih.exit
  %barrier.i13 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 0
  %8 = load i8, i8* %barrier.i13, align 1, !tbaa !9
  %9 = lshr i8 %8, 3
  %barrier1.i14 = getelementptr inbounds %struct.Object, %struct.Object* %6, i64 0, i32 0
  %10 = load i8, i8* %barrier1.i14, align 1, !tbaa !9
  %or20.i15 = or i8 %9, %10
  %and421.i16 = and i8 %or20.i15, %barrier_mask
  %cmp5.i17 = icmp eq i8 %and421.i16, 0
  br i1 %cmp5.i17, label %if.then7.i19, label %_ZL16AllocateAndStoreP6Objectih.exit20, !prof !10

if.then7.i19:                                     ; preds = %if.end.i18
  tail call void asm sideeffect "", "{x0},{x0},{x26},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"(%struct.Object* nonnull %0, %struct.Object* nonnull %0, %struct.Object** nonnull %arrayidx.i10) #1, !srcloc !11
  br label %_ZL16AllocateAndStoreP6Objectih.exit20

_ZL16AllocateAndStoreP6Objectih.exit20:           ; preds = %_ZL16AllocateAndStoreP6Objectih.exit, %if.end.i18, %if.then7.i19
  %11 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %arrayidx.i21 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 4, i64 2
  store %struct.Object* %11, %struct.Object** %arrayidx.i21, align 1, !tbaa !5
  %12 = ptrtoint %struct.Object* %11 to i64
  %and.i22 = and i64 %12, 1
  %cmp.i23 = icmp eq i64 %and.i22, 0
  br i1 %cmp.i23, label %_ZL16AllocateAndStoreP6Objectih.exit31, label %if.end.i29

if.end.i29:                                       ; preds = %_ZL16AllocateAndStoreP6Objectih.exit20
  %barrier.i24 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 0
  %13 = load i8, i8* %barrier.i24, align 1, !tbaa !9
  %14 = lshr i8 %13, 3
  %barrier1.i25 = getelementptr inbounds %struct.Object, %struct.Object* %11, i64 0, i32 0
  %15 = load i8, i8* %barrier1.i25, align 1, !tbaa !9
  %or20.i26 = or i8 %14, %15
  %and421.i27 = and i8 %or20.i26, %barrier_mask
  %cmp5.i28 = icmp eq i8 %and421.i27, 0
  br i1 %cmp5.i28, label %if.then7.i30, label %_ZL16AllocateAndStoreP6Objectih.exit31, !prof !10

if.then7.i30:                                     ; preds = %if.end.i29
  tail call void asm sideeffect "", "{x0},{x0},{x26},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"(%struct.Object* nonnull %0, %struct.Object* nonnull %0, %struct.Object** nonnull %arrayidx.i21) #1, !srcloc !11
  br label %_ZL16AllocateAndStoreP6Objectih.exit31

_ZL16AllocateAndStoreP6Objectih.exit31:           ; preds = %_ZL16AllocateAndStoreP6Objectih.exit20, %if.end.i29, %if.then7.i30
  %16 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %arrayidx.i32 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 4, i64 3
  store %struct.Object* %16, %struct.Object** %arrayidx.i32, align 1, !tbaa !5
  %17 = ptrtoint %struct.Object* %16 to i64
  %and.i33 = and i64 %17, 1
  %cmp.i34 = icmp eq i64 %and.i33, 0
  br i1 %cmp.i34, label %_ZL16AllocateAndStoreP6Objectih.exit42, label %if.end.i40

if.end.i40:                                       ; preds = %_ZL16AllocateAndStoreP6Objectih.exit31
  %barrier.i35 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 0
  %18 = load i8, i8* %barrier.i35, align 1, !tbaa !9
  %19 = lshr i8 %18, 3
  %barrier1.i36 = getelementptr inbounds %struct.Object, %struct.Object* %16, i64 0, i32 0
  %20 = load i8, i8* %barrier1.i36, align 1, !tbaa !9
  %or20.i37 = or i8 %19, %20
  %and421.i38 = and i8 %or20.i37, %barrier_mask
  %cmp5.i39 = icmp eq i8 %and421.i38, 0
  br i1 %cmp5.i39, label %if.then7.i41, label %_ZL16AllocateAndStoreP6Objectih.exit42, !prof !10

if.then7.i41:                                     ; preds = %if.end.i40
  tail call void asm sideeffect "", "{x0},{x0},{x26},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"(%struct.Object* nonnull %0, %struct.Object* nonnull %0, %struct.Object** nonnull %arrayidx.i32) #1, !srcloc !11
  br label %_ZL16AllocateAndStoreP6Objectih.exit42

_ZL16AllocateAndStoreP6Objectih.exit42:           ; preds = %_ZL16AllocateAndStoreP6Objectih.exit31, %if.end.i40, %if.then7.i41
  %21 = tail call %struct.Object* asm sideeffect "", "={x0},~{x1},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"() #1, !srcloc !4
  %arrayidx.i43 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 4, i64 4
  store %struct.Object* %21, %struct.Object** %arrayidx.i43, align 1, !tbaa !5
  %22 = ptrtoint %struct.Object* %21 to i64
  %and.i44 = and i64 %22, 1
  %cmp.i45 = icmp eq i64 %and.i44, 0
  br i1 %cmp.i45, label %_ZL16AllocateAndStoreP6Objectih.exit53, label %if.end.i51

if.end.i51:                                       ; preds = %_ZL16AllocateAndStoreP6Objectih.exit42
  %barrier.i46 = getelementptr inbounds %struct.Object, %struct.Object* %0, i64 0, i32 0
  %23 = load i8, i8* %barrier.i46, align 1, !tbaa !9
  %24 = lshr i8 %23, 3
  %barrier1.i47 = getelementptr inbounds %struct.Object, %struct.Object* %21, i64 0, i32 0
  %25 = load i8, i8* %barrier1.i47, align 1, !tbaa !9
  %or20.i48 = or i8 %24, %25
  %and421.i49 = and i8 %or20.i48, %barrier_mask
  %cmp5.i50 = icmp eq i8 %and421.i49, 0
  br i1 %cmp5.i50, label %if.then7.i52, label %_ZL16AllocateAndStoreP6Objectih.exit53, !prof !10

if.then7.i52:                                     ; preds = %if.end.i51
  tail call void asm sideeffect "", "{x0},{x0},{x26},~{x2},~{x3},~{x4},~{x5},~{x6},~{x7},~{x8},~{x9},~{x10},~{x11},~{x12},~{x13},~{x14},~{x15},~{x16},~{x17},~{x18},~{x19},~{x20},~{x21},~{x22},~{x23},~{x24},~{x25},~{x27},~{x28},~{fp},~{lr}"(%struct.Object* nonnull %0, %struct.Object* nonnull %0, %struct.Object** nonnull %arrayidx.i43) #1, !srcloc !11
  br label %_ZL16AllocateAndStoreP6Objectih.exit53

_ZL16AllocateAndStoreP6Objectih.exit53:           ; preds = %_ZL16AllocateAndStoreP6Objectih.exit42, %if.end.i51, %if.then7.i52
  ret void
}

attributes #0 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="generic" "target-features"="+neon" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind }

!llvm.module.flags = !{!0, !1, !2}
!llvm.ident = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"PIC Level", i32 2}
!2 = !{i32 7, !"PIE Level", i32 2}
!3 = !{!"clang version 9.0.0 (tags/RELEASE_900/final 375507)"}
!4 = !{i32 299}
!5 = !{!6, !6, i64 0}
!6 = !{!"any pointer", !7, i64 0}
!7 = !{!"omnipotent char", !8, i64 0}
!8 = !{!"Simple C++ TBAA"}
!9 = !{!7, !7, i64 0}
!10 = !{!"branch_weights", i32 1, i32 2000}
!11 = !{i32 1000}
