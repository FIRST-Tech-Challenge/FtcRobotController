package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public abstract class UltimateGoalHardware extends RobotHardware {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor shooterLeft;
    public DcMotor shooterRight;
    public DcMotor collector;
    public DcMotor escalator;

    public Servo wobbleGoalHolder;

    public final static double COUNTS_PER_ENCODER_REV = 8192;
    public final static double WHEEL_DIAMETER_IN = 4;

    @Override
    public void initializeHardware() {
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        shooterLeft = this.initializeDevice(DcMotor.class, "shooterLeft");
        shooterRight = this.initializeDevice(DcMotor.class, "shooterRight");
        collector = this.initializeDevice(DcMotor.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        escalator = this.initializeDevice(DcMotor.class, "escalator");
        wobbleGoalHolder = this.initializeDevice(Servo.class, "wobble");

        this.initializeOmniDrive(frontLeft, frontRight, backLeft, backRight);
        this.omniDrive.setCountsPerInch((Math.PI*WHEEL_DIAMETER_IN)/COUNTS_PER_ENCODER_REV);
    }
}
