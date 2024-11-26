package com.kalipsorobotics.actions.sequences;

import android.graphics.Path;
import android.os.SystemClock;

import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
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
    Intake intake;
    IntakeNoodleAction intakeNoodleAction;
    IntakeDoorAction intakeDoorAction;

    public TransferSequence(HardwareMap hardwareMap, OpModeUtilities opModeUtilities, Outtake outtake, Intake intake) {
        this.hardwareMap = hardwareMap;
        this.opModeUtilities = opModeUtilities;
        this.outtake = outtake;
        this.intake = intake;
        this.outtakeSlideAction = new OuttakeSlideAction(outtake);
        this.intakeDoorAction = new IntakeDoorAction(intake);
        this.intakeNoodleAction = new IntakeNoodleAction(intake);
    }
    public sequence() {
        outtakeSlideAction.moveToPosition(400);
        intakeNoodleAction.stop();
        intakeDoorAction.open();
        SystemClock.sleep(500);
        intakeNoodleAction.run();
        SystemClock.sleep(800);
        intakeDoorAction.close();
        intakeNoodleAction.stop();
        SystemClock.sleep(600);
        outtakeSlideAction.down();
        intakePivotAction.moveDown();
        SystemClock.sleep(800);
        outtakeClawAction.open();
        outtakePivotAction.setPosition(0.925);
        SystemClock.sleep(330);
        outtakeClawAction.close();
        SystemClock.sleep(1000);
        outtakeSlideAction.Toggle();
        SystemClock.sleep(700);
        outtakeSlideAction.idle();
    }
}
