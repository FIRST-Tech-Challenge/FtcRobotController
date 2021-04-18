package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Ultimate Goal Accessory
 *
 * @author  Nikhil
 * @version 2.0
 * @since   2020-October-26
 *
 */
public class  Shooter {
    protected LinearOpMode op = null;

    private DcMotorEx shooterMotor;

    Servo shooter_Servo;

    protected double highGoalVelocity = 1675;
    protected double middleGoalVelocity = 1600;
    protected double lowGoalVelocity = 1500;
    protected double powershotVelocity = 1725;
    protected double veloThreshold = 50;
    double servoBack;
    double servoForward;

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
            shooterMotor.setVelocityPIDFCoefficients(17, 0, 0.125, 17.75);//17,0,0.15,16.5
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
        double rotationsPerMinute = rotationsPerSecond * 60;
        return rotationsPerMinute;
    }


    public void shoot(double speed, int distance, int rings) {
//        ElapsedTime runtime = new ElapsedTime();
//        op.telemetry.addData("speed: ", getRPM());
//        op.telemetry.update();
//        op.sleep(100);
        setVelocity(speed, distance);
        op.sleep(1000);
        if (shooterMotor.getVelocity() > 0) {
//            op.sleep(100);
//            op.telemetry.clear();
//            op.telemetry.addData("status", getRPM());
//            op.telemetry.update();
        }
        for (int i = 0; i < rings; i++) {
            moveServo(false);
            moveServo(true);
            op.sleep(25);
        }
        if(op.getRuntime()>3){
            stopShooter();
        }
    }

    public void moveServo(boolean direction) {
        if (direction == true) {
            shooter_Servo.setPosition(servoBack);
        } else {
            shooter_Servo.setPosition(servoForward);
        }
        op.telemetry.addData("pusher position :", direction);
        op.telemetry.update();
        op.sleep(90);//170
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

