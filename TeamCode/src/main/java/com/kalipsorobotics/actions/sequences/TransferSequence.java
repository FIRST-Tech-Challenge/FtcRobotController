package com.kalipsorobotics.actions.sequences;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
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
    IntakeSequence intakeSequence;


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
        this.intakeSequence = new IntakeSequence(intakePivotAction, new IntakeLinkageAction(intake));
    }
//    public boolean checkdone(double time, double waitLength) {
//        if (SystemClock.currentThreadTimeMillis() - time > waitLength) {
//            return true;
//        } else  {
//            return false;
//        }
//    }
    public void sequence() {
        outtakePivotAction.moveOut();
        intakeNoodleAction.run();
        outtakeSlideAction.moveToPosition(300);
        outtakeClawAction.open();
        intakeDoorAction.open();
        outtakeClawAction.open();

        SystemClock.sleep(700);

        outtakeSlideAction.down();
        intakeNoodleAction.stop();
        intakePivotAction.moveDown();
        outtakePivotAction.moveIn();
        SystemClock.sleep(650);
        outtakeClawAction.close();
        //ADD LINEAR SLIDE STUFF
    }
}

