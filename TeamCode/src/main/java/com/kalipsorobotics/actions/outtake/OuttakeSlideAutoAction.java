//package com.kalipsorobotics.actions.outtake;
//
//import androidx.savedstate.SavedStateRegistryOwner;
//
//import com.kalipsorobotics.actions.Action;
//import com.kalipsorobotics.actions.ActionSet;
//import com.kalipsorobotics.actions.KActionSet;
//import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
//import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
//import com.kalipsorobotics.modules.Outtake;
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class OuttakeSlideAutoAction extends Action {
//    MoveLSAction moveLSAction;
//
//    public OuttakeSlideAutoAction(Outtake outtake, double targetPos) {
//            moveLSAction = new MoveLSAction(targetPos, outtake);
//    }
//
//    @Override
//    public boolean checkDoneCondition() {
//        return getIsDone();
//    }
//}