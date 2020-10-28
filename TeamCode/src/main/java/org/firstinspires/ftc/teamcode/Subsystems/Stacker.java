package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stacker {

    public Servo leftFliper = null;
    public Servo rightFliper = null;
    public Servo stacker = null;

    // Constants
    private static final double leftUp = 0.75; // .75 a little shy but ok due to hitting bolt
    private static final double leftBack = 0.4; //good at 0.4;
    private static final double rightUp = (1-leftUp);
    private static final double rightBack = (1-leftBack);
    private static final double flippercenter = 0.5;

    public void init(HardwareMap hwMap) {
        leftFliper = hwMap.get(Servo.class, "Left_Flipper");
        rightFliper = hwMap.get(Servo.class, "Right_Flipper");
        stacker =  hwMap.get(Servo.class, "Stacker");
    }
        //x button
    public void loadShooter() {
        leftFliper.setPosition(leftUp);
        rightFliper.setPosition(rightUp);
    }
    public void resetShooter() {
        leftFliper.setPosition(leftBack);
        rightFliper.setPosition(rightBack);
    }
    public void flipperCalibrateinCenter() {
        leftFliper.setPosition(flippercenter);
        rightFliper.setPosition(flippercenter);
    }
}
