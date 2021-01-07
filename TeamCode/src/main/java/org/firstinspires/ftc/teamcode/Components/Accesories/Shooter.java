package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Ultimate Goal Accessory
 *
 * @author  Nikhil
 * @version 2.0
 * @since   2020-October-26
 *
 */
public class Shooter {
    protected LinearOpMode op = null;

//TODO Make private when warren removes his function from robot class @author
    public DcMotorEx shooterMotor;

    Servo shooter_Servo;

    //TODO adjust based on testing
    protected double highGoalVelocity = 1675;
    protected double middleGoalVelocity = 1600;
    protected double lowGoalVelocity = 1500;

    public Shooter(LinearOpMode opMode) {
        op = opMode;

        shooterMotor = (DcMotorEx) op.hardwareMap.dcMotor.get("ShooterMotor");//gets the name ShooterMotor from hardware map and assigns it to shooter_Motor
        shooter_Servo = op.hardwareMap.servo.get("ShooterServo");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(57, 0, 0, 17);
        shooter_Servo.setPosition(1.0);
    }

    private void setVelocity(double velocity, int distance) {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setVelocity(velocity);
        shooterMotor.setTargetPosition(distance);

    }

    public void stopShooter() {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setVelocity(0);
        shooterMotor.setTargetPosition(0);
    }

    private double getRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();
        double rotationsPerSecond = ticksPerSecond / 28;
        double rotationsPerMinute = rotationsPerSecond * 60;
        return rotationsPerMinute;
    }


    public void shoot(double speed, int distance, int rings) {
        ElapsedTime runtime = new ElapsedTime();
        op.telemetry.addData("speed: ", getRPM());
        op.telemetry.update();
        op.sleep(100);
        setVelocity(speed, distance);
        op.sleep(1500);
        if (shooterMotor.getVelocity() > 0) {
            op.sleep(100);
            op.telemetry.clear();
            op.telemetry.addData("status", getRPM());
            op.telemetry.update();
        }
        for (int i = 0; i < rings; i++) {
            moveServo(false);
            moveServo(true);
        }
        if(op.getRuntime()>3){
            stopShooter();
        }

    }


    public void moveServo(boolean direction) {
        if (direction == true) {
            shooter_Servo.setPosition(1.0);
        } else {
            shooter_Servo.setPosition(0.0);
        }
        op.telemetry.addData("pusher position :", direction);
        op.telemetry.update();
        op.sleep(500);
    }


    public void shootGoalTeleop(int distance) {
        setVelocity(highGoalVelocity, distance);
    }

    public void shootHighGoal(int rings) {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot(highGoalVelocity, 1, 3);
    }

    public void shootMidGoal(int rings) {
        shoot(middleGoalVelocity, 1000, 3);

    }

    public void shootLowGoal(int rings) {
        shoot(lowGoalVelocity, 1000, 3);
        }

    }

