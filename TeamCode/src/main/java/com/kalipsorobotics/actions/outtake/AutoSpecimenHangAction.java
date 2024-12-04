//package com.kalipsorobotics.actions.outtake;
//
//import com.kalipsorobotics.actions.Action;
//import com.kalipsorobotics.modules.Outtake;
//
//public class AutoSpecimenHangAction extends Action {
//    Outtake outtake;
//    MoveLSAction moveLSUp;
//    MoveLSAction moveLSDown;
//    OuttakePivotAutoAction outtakePivotAction;
//
//    public AutoSpecimenHangAction(Outtake outtake) {
//        this.outtake = outtake;
//        moveLSUp = new MoveLSAction(30, outtake);
//        outtakePivotAction = new OuttakePivotAutoAction(outtake, OuttakePivotAutoAction.Position.SPECIMEN);
//        moveLSDown = new MoveLSAction(0, outtake, 0.01);
//        moveLSDown.setDependentAction(outtakePivotAction);
//
//    }
//
//    @Override
//    public boolean checkDoneCondition() {
//        if(moveLSDown.getIsDone()) {
//            return true;
//        } else {
//            return false;
//        }
//    }
//
//    @Override
//    public void update() {
//        moveLSUp.updateCheckDone();
//        outtakePivotAction.updateCheckDone();
//        moveLSDown.updateCheckDone();
//    }
//}
