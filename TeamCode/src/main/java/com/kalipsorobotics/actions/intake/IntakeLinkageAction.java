package com.kalipsorobotics.actions.intake;

import android.os.SystemClock;

import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Time;

//0.6 closed
//0.245 opened
public class IntakeLinkageAction {

    private final Intake intake;
    private final Servo linkageServo1;
    private final Servo linkageServo2;
    private double startTime;
 //   Time time = new Time(0,0,1);


    private boolean isRetracted = true;

    public IntakeLinkageAction(Intake intake) {
        this.intake = intake;
        this.linkageServo1 = intake.getLinkageServo1();
        this.linkageServo2 = intake.getLinkageServo2();
        linkageServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void moveIntakeSlide(double position) {
        linkageServo1.setPosition(-position);
        linkageServo2.setPosition(-position);
    }
    public double getPosition() {
        return linkageServo1.getPosition();
    }

    public boolean checkExtend() {
        double endTime = SystemClock.currentThreadTimeMillis();
        return endTime - startTime > 1500;
    }
    public void control(double joystick) {
        if (joystick > 0.1) {
            setPosition(getPosition()+0.1);
        } else if (joystick < -0.1) {
            setPosition(getPosition()-0.1);
        }
    }
    public void setPosition(double position) {
        linkageServo1.setPosition(position);
        linkageServo2.setPosition(position);
    }
    public void extend() {
        startTime = SystemClock.currentThreadTimeMillis();
        moveIntakeSlide(0);
        isRetracted = false;
    }

    public void retract() {
        //original 0.7
        moveIntakeSlide(-1);
        isRetracted = true;
    }

    public void togglePosition() {
        if (!isRetracted) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isRetracted() {
        return isRetracted;
    }

    public Intake getIntake() {
        return intake;
    }
}
