package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;

public class IntakeTransferAction extends KActionSet {

    public IntakeTransferAction(Intake intake, Outtake outtake) {
        KServoAutoAction intakePivotDown = new KServoAutoAction(intake.getIntakePivotServo(),
                IntakePivotAction.INTAKE_PIVOT_DOWN_POS);
        intakePivotDown.setName("intakePivotDown");
        this.addAction(intakePivotDown);

        MoveIntakeLSAction moveIntakeLSIn = new MoveIntakeLSAction(intake, 0);
        moveIntakeLSIn.setName("moveIntakeLSIn");
        this.addAction(moveIntakeLSIn);

//        KServoAutoAction outtakePivotOut = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                OuttakePivotAction.OUTTAKE_PIVOT_OUT_POS);
//        outtakePivotOut.setName("outtakePivotOut");
//        this.addAction(outtakePivotOut);

        MoveOuttakeLSAction moveOuttakeLSUp = new MoveOuttakeLSAction(outtake, 100);
        moveOuttakeLSUp.setName("moveOuttakeLSUp");
        this.addAction(moveOuttakeLSUp);

        KServoAutoAction intakePivotUp = new KServoAutoAction(intake.getIntakePivotServo(),
                IntakePivotAction.INTAKE_PIVOT_UP_POS);
        intakePivotUp.setName("intakePivotUp");
        this.addAction(intakePivotUp);
        setDependantActions(moveIntakeLSIn);

        IntakeNoodleAction intakeNoodleActionRun = new IntakeNoodleAction(intake, 200, true);
        intakeNoodleActionRun.setName("intakeNoodleActionRun");
        this.addAction(intakeNoodleActionRun);
        intakeNoodleActionRun.setDependantActions(intakePivotUp);

        KServoAutoAction intakeDoorOpen = new KServoAutoAction(intake.getDoorServo(),
                IntakeDoorAction.INTAKE_DOOR_OPEN_POS);
        intakeDoorOpen.setName("intakeDoorOpen");
        this.addAction(intakeDoorOpen);
        intakeDoorOpen.setDependantActions(intakePivotUp);

        MoveOuttakeLSAction moveOuttakeLSDown = new MoveOuttakeLSAction(outtake, 0);
        moveOuttakeLSDown.setName("moveOuttakeLSDown");
        this.addAction(moveOuttakeLSDown);
        moveOuttakeLSDown.setDependantActions(intakeDoorOpen);

    }

}
