package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.AutoActions.KServoAutoAction;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;

public class IntakeTransferAction extends KActionSet {

    public IntakeTransferAction(Intake intake, Outtake outtake) {
        KServoAutoAction intakePivotDown = new KServoAutoAction(intake.getIntakePivotServo(),
                IntakePivotAction.INTAKE_PIVOT_DOWN_POS);
        intakePivotDown.setName("intakePivotDown");

        MoveIntakeLSAction moveIntakeLSIn = new MoveIntakeLSAction(intake, 0);
        moveIntakeLSIn.setName("moveIntakeLSIn");

        KServoAutoAction outtakePivotOut = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_OUT_POS);
        outtakePivotOut.setName("outtakePivotOut");
        this.addAction(outtakePivotOut);

        KServoAutoAction intakePivotUp = new KServoAutoAction(intake.getIntakePivotServo(),
                IntakePivotAction.INTAKE_PIVOT_UP_POS);
        intakePivotUp.setName("intakePivotUp");
        setDependantActions(moveIntakeLSIn);

        IntakeNoodleAction intakeNoodleActionRun = new IntakeNoodleAction(intake, 200, true);
        intakeNoodleActionRun.setName("intakeNoodleActionRun");
        intakeNoodleActionRun.setDependantActions(intakePivotUp);

        KServoAutoAction intakeDoorOpen = new KServoAutoAction(intake.getDoorServo(),
                IntakeDoorAction.INTAKE_DOOR_OPEN_POS);
        intakeDoorOpen.setName("intakeDoorOpen");
        intakeDoorOpen.setDependantActions(intakeNoodleActionRun);



    }

}
