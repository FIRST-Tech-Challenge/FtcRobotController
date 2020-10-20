package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stacker {

    public Servo leftFliper = null;
    public Servo rightFliper = null;

    // Constants
    private static final double leftUp = .5;
    private static final double leftBack = .6;
    private static final double rightUp = .5;
    private static final double rightBack = .6;

    public void init(HardwareMap hwMap) {
        leftFliper = hwMap.get(Servo.class, "ForwardMove");
        rightFliper = hwMap.get(Servo.class, "BackMove");
    }

    public void loadShooter() {
        leftFliper.setPosition(leftUp);
        rightFliper.setPosition(rightUp);
    }
    public void resetShooter() {
        leftFliper.setPosition(leftBack);
        rightFliper.setPosition(rightBack);
    }
}
