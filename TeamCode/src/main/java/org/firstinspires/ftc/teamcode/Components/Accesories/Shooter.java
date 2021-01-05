package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Ultimate Goal Accessory
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
public class Shooter {
    protected LinearOpMode op = null;


    public DcMotorEx shooterMotor;

    public Servo shooter_Servo;

    //velocity
    protected double highGoalVelocity = 1850;
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

    public void setVelocity(double velocity, int distance) {
        shooterMotor.setVelocity(velocity);
        shooterMotor.setTargetPosition(distance);

    }

    public void stopShooter() {
        shooterMotor.setVelocity(0);
    }

    public double getRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();
        double rotationsPerSecond = ticksPerSecond / 28;
        double rotationsPerMinute = rotationsPerSecond * 60;
        return rotationsPerMinute;
    }


    public void shoot(double speed, int distance, int rings) {
        op.telemetry.addData("speed: ", getRPM());
        op.telemetry.update();
        op.sleep(3000);
        setVelocity(speed, distance);
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

        if (shooterMotor.getTargetPosition()>=distance){
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
        shoot(highGoalVelocity, 1000, 3);
        op.sleep(1000);
    }

    public void shootMidGoal(int rings) {
        shoot(highGoalVelocity, 1000, 3);
        op.sleep(1000);
        for (int i = 0; i < rings; i++) {
            moveServo(false);
            moveServo(true);
        }

    }

    public void shootLowGoal(int rings) {
        shoot(highGoalVelocity, 1000, 3);
        op.sleep(1000);
        for (int i = 0; i < rings; i++) {
            moveServo(false);
            moveServo(true);
        }

    }

}
