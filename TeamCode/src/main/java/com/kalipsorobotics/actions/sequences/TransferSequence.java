package com.kalipsorobotics.actions.sequences;

import android.os.SystemClock;

import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeSlideAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSequence {
    private final HardwareMap hardwareMap;
    OpModeUtilities opModeUtilities;
    Outtake outtake;
    OuttakeSlideAction outtakeSlideAction;
    OuttakeClawAction outtakeClawAction;
    Intake intake;
    IntakeNoodleAction intakeNoodleAction;
    IntakeDoorAction intakeDoorAction;
    IntakePivotAction intakePivotAction;
    OuttakePivotAction outtakePivotAction;


    public TransferSequence(HardwareMap hardwareMap, OpModeUtilities opModeUtilities, Outtake outtake, Intake intake) {
        this.hardwareMap = hardwareMap;
        this.opModeUtilities = opModeUtilities;
        this.outtake = outtake;
        this.outtakeClawAction = new OuttakeClawAction(outtake);
        this.outtakePivotAction = new OuttakePivotAction(outtake);
        this.intake = intake;
        this.outtakeSlideAction = new OuttakeSlideAction(outtake);
        this.intakeDoorAction = new IntakeDoorAction(intake);
        this.intakeNoodleAction = new IntakeNoodleAction(intake);
        this.intakePivotAction = new IntakePivotAction(intake);
        this.outtakeClawAction = new OuttakeClawAction(outtake);
        this.outtakePivotAction = new OuttakePivotAction(outtake);
    }
    public void sequence() {
        intakeNoodleAction.stop();
        intakeDoorAction.open();
        SystemClock.sleep(500);
        intakeNoodleAction.run();
        SystemClock.sleep(800);
        intakeDoorAction.close();
        intakeNoodleAction.stop();
        SystemClock.sleep(600);
        intakePivotAction.moveDown();
        SystemClock.sleep(800);
        outtakePivotAction.moveIn();
    }
}
