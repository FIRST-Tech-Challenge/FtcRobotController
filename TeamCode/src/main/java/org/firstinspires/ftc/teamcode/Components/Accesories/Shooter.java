package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

/*
 * Ultimate Goal Accessory
 *
 * @author  Nikhil
 * @version 2.0
 * @since   2020-October-26
 *
 */
public class  Shooter {
    protected LinearOpMode op = null;

    private final DcMotorEx shooterMotor;

    final Servo shooter_Servo;

    protected final double highGoalVelocity = 1675;
    protected final double middleGoalVelocity = 1600;
    protected final double lowGoalVelocity = 1500;
    protected final double powershotVelocity = 1725;
    protected double veloThreshold = 50;
    final double servoBack;
    final double servoForward;

    public Shooter(LinearOpMode opMode) {
        op = opMode;

        shooterMotor = (DcMotorEx) op.hardwareMap.dcMotor.get("ShooterMotor");//gets the name ShooterMotor from hardware map and assigns it to shooter_Motor
        shooter_Servo = op.hardwareMap.servo.get("ShooterServo");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if(!Robot.isCorgi){
            servoBack = 0.59;
            servoForward = 0.52;
            shooterMotor.setVelocityPIDFCoefficients(57, 0, 0, 15.4);
        }
        else{
            servoBack = 0.64;
            servoForward = 0.5;
            shooterMotor.setVelocityPIDFCoefficients(17, 0, 0.125, 18.5);
        }
        shooter_Servo.setPosition(servoBack);

    }

    public void setVelocity(double velocity, int distance) {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setVelocity(velocity);
        shooterMotor.setTargetPosition(distance);

    }

    public void stopShooter() {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setVelocity(0);
        shooterMotor.setTargetPosition(0);
    }

    public double getRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();
        double rotationsPerSecond = ticksPerSecond / 28;
        return rotationsPerSecond * 60;
    }


    public void shoot(double speed, int distance, int rings) {
//        ElapsedTime runtime = new ElapsedTime();
//        op.telemetry.addData("speed: ", getRPM());
//        op.telemetry.update();
//        op.sleep(100);
        setVelocity(speed, distance);
        op.sleep(750);
        if (shooterMotor.getVelocity() > 0) {
//            op.sleep(100);
//            op.telemetry.clear();
//            op.telemetry.addData("status", getRPM());
//            op.telemetry.update();
        }
        for (int i = 0; i < rings; i++) {
            if(i>0) {
                op.sleep(130);
            }
            moveServo(false);
            moveServo(true);
        }
        return;
    }

    public void moveServo(boolean direction) {
        if (direction) {
            shooter_Servo.setPosition(servoBack);
        } else {
            shooter_Servo.setPosition(servoForward);
        }
        op.telemetry.addData("pusher position :", direction);
        op.telemetry.update();
        op.sleep(130);//130
    }

    public void shootGoalTeleop(int distance) {
        setVelocity(highGoalVelocity, distance);
    }

    public void shootHighGoal(int rings) {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot(highGoalVelocity, 1, rings);
    }

    public void shootHighGoalTest(double speed, int distance, int rings) {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot(speed, distance, rings);
    }

    public void shootMidGoal(int rings) {
        shoot(middleGoalVelocity, 1000, 3);
    }

    public void shootLowGoal(int rings) {
        shoot(lowGoalVelocity, 1000, 3);
        }

    public void shootPowershot (int rings){
    shoot(powershotVelocity, 1000, 3);
    }
}

