package com.kalipsorobotics.actions.intake;

import static android.os.SystemClock.sleep;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.modules.ColorDetector;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class IntakeAction extends Action {
    OpModeUtilities opModeUtilities;
    Intake intake;
    IntakeLinkageAction intakeLinkageAction;
    IntakePivotAction intakePivotAction;
    IntakeDoorAction intakeDoorAction;
    IntakeNoodleAction intakeNoodleAction;
    ColorDetector colorDetector;

    public IntakeAction(OpModeUtilities opModeUtilities, Intake intake) {
        this.opModeUtilities = opModeUtilities;
        this.intake = intake;
        this.intakeLinkageAction = new IntakeLinkageAction(intake);
        this.intakePivotAction = new IntakePivotAction(intake);
        this.intakeDoorAction = new IntakeDoorAction(intake);
        this.intakeNoodleAction = new IntakeNoodleAction(intake);
        this.colorDetector = new ColorDetector(opModeUtilities);
    }
    @Override
    public boolean checkDoneCondition() {
        return false;
    }
    public void slideTo(double position) {
        intakeLinkageAction.moveIntakeSlide(position);
        Log.d("intake slide", "slide has been moved to " + position);
    }
    public void extendSlide() {
        intakeLinkageAction.extend();
        Log.d("intake slide", "slide has extended");
    }
    public void retractSlide() {
        intakeLinkageAction.retract();
        Log.d("intake slide", "slide has retracted");
    }
    public void noodleCycle(boolean isRed, double seconds) {
        if (((!isRed) && colorDetector.detectColor() == KColor.Color.RED) ||
                ((isRed) && colorDetector.detectColor() == KColor.Color.BLUE)) {
            intakeNoodleAction.reverse();
            sleep(1000);
        } else {
            intakeNoodleAction.run();
        }
    }
    public void doorToggle() {
        intakeDoorAction.togglePosition();
    }
    public void intakePivotToggle() {
        intakePivotAction.togglePosition();
    }
    public void IntakeEngage() {
        intakeLinkageAction.extend();
        sleep(1200);
        intakePivotAction.moveDown();
    }
    public void IntakeUnengage() {
        intakeLinkageAction.retract();
        intakePivotAction.moveDown();
    }
    public void transferToPlate() {
        intakeNoodleAction.run();
        intakeDoorAction.open();
        sleep(900);
        intakeDoorAction.close();
        intakePivotToggle();
    }
}