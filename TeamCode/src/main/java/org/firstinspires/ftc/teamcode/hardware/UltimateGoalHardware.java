package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public abstract class UltimateGoalHardware extends RobotHardware {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor shooterLeft;
    DcMotor shooterRight;
    DcMotor collector;
    DcMotor escalator;

    Servo wobbleGoalHolder;

    @Override
    public void initializeHardware() {
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        shooterLeft = this.initializeDevice(DcMotor.class, "shooterLeft");
        shooterRight = this.initializeDevice(DcMotor.class, "shooterRight");
        collector = this.initializeDevice(DcMotor.class, "collector");
        escalator = this.initializeDevice(DcMotor.class, "escalator");
        wobbleGoalHolder = this.initializeDevice(Servo.class, "wobble");
    }
}
