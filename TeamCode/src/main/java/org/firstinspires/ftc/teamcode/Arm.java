package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    DcMotor arm = null;
    CRServo hand = null;
//    Servo wrist = null;
    double targetPosition = 0;
    double kP = .001;
    public void init (HardwareMap hwMap) {
        arm = hwMap.get(DcMotor.class, "arm");
        hand = hwMap.get(CRServo.class, "hand");
//        wrist = hwMap.get(Servo.class, "wrist");
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void spinOut() {
        hand.setPower(1);
    }
    void spinIn() {
        hand.setPower(-1);
    }
    void spinOff() {
        hand.setPower(0);
    }

    void spinAtPower(double power) {
        hand.setPower(power);
    }

    void updateArm() {
        double error = targetPosition - arm.getCurrentPosition();

        double power = error * kP;
        arm.setPower(power);
//        wrist.setPosition(0);
    }

    void setPosition(double target) {
        targetPosition = target;
    }
}

