package com.kalipsorobotics.actions.intake;

import static android.os.SystemClock.sleep;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.modules.ColorDetector;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeAction extends Action {
    DcMotor intakeMotor;
    ColorDetector colorDetector;

    public IntakeAction(Intake intake) {
        this.intakeMotor = intake.getNoodleMotor();
        this.colorDetector = new ColorDetector(intake.getOpModeUtilities(), intake.getOpModeUtilities().getHardwareMap());
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }
        if(colorDetector.detectColor() == KColor.Color.YELLOW) {
            intakeMotor.setPower(0);
            isDone = true;
        } else if(colorDetector.detectColor() == KColor.Color.NONE) {
            intakeMotor.setPower(1);
        }
        super.update();
    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }


    /*public void slideTo(double position) {
        //intakeLinkageAction.moveIntakeSlide(position);
        Log.d("intake slide", "slide has been moved to " + position);
    }
    public void extendSlide() {
        //intakeLinkageAction.extend();
        Log.d("intake slide", "slide has extended");
    }
    public void retractSlide() {
        //intakeLinkageAction.retract();
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
        //intakeLinkageAction.extend();
        sleep(1200);
        intakePivotAction.moveDown();
    }
    public void IntakeUnengage() {
        //intakeLinkageAction.retract();
        intakePivotAction.moveDown();
    }
    public void transferToPlate() {
        intakeNoodleAction.run();
        intakeDoorAction.open();
        sleep(900);
        intakeDoorAction.close();
        intakePivotToggle();
    }*/
}