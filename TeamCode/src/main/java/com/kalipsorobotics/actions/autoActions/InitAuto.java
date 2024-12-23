package com.kalipsorobotics.actions.autoActions;

//import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.intake.MoveIntakeLSAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;

public class InitAuto extends KActionSet {

    public InitAuto(Intake intake, Outtake outtake) {

        MoveIntakeLSAction moveIntakeLSAction = new MoveIntakeLSAction(intake, -5);
        moveIntakeLSAction.setName("moveIntakeLSAction");
        this.addAction(moveIntakeLSAction);

        KServoAutoAction intakePivotUp = new KServoAutoAction(intake.getIntakePivotServo(),
                IntakePivotAction.INTAKE_PIVOT_UP_POS);
        intakePivotUp.setName("intakePivotUp");
        this.addAction(intakePivotUp);

        KServoAutoAction clawClose = new KServoAutoAction(outtake.getOuttakeClaw(),
                OuttakeClawAction.OUTTAKE_CLAW_CLOSE_POS);
        clawClose.setName("clawClose");
        this.addAction(clawClose);

    }

}
