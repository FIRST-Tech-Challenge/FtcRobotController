package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;

/**
 * Single shooter flywheel motor + the CR servo that feeds balls into it.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Shooter {

    private final DcMotorEx shooter;
    private double targetPower;
    private final CRServo feeder;
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence cadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(Constants.SPINDEXER_FEED_PULSE_SEC,Constants.SPINDEXER_SHOT_GAP_SEC);
    private double now(){return System.nanoTime()/1e9;}

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);
        feeder = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setDirection(CRServo.Direction.REVERSE);

        stop();
    }

    public void spinUp() {
        apply(Constants.SHOOTER_POWER,0);
    }

    /** RobotIO.shooterPower is an ordinary open-loop motor power command. */
    public void apply(double shooterPower,double feedPower) {
        double scale=Double.isFinite(shooterPower)?Math.max(-1,Math.min(1,shooterPower)):0;
        targetPower=scale;
        shooter.setPower(scale);
        feeder.setPower(cadence.apply(scale>0?feedPower:0,now())*Constants.SHOOTER_SERVO_POWER);
    }

    public double getTargetPower() { return targetPower; }

    public double getAppliedPower() { return shooter.getPower(); }

    public void feed() {
        feeder.setPower(cadence.apply(1,now())*Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        cadence.apply(0,now());
        feeder.setPower(0);
    }

    public void stop() {
        targetPower=0;
        cadence.apply(0,now());
        SafeShutdown.run(()->shooter.setPower(0),()->feeder.setPower(0));
    }
}
