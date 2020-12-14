package org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Ultimate Goal Accessory
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
public class Shooter {
    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    public DcMotorEx shooterMotor;
    Servo shooter_Servo;
    private double speedTopGoal = 1;//will get changed when testing
    private double speedMediumGoal=0.5;//will get changed when testing
    private double speedLowGoal=0.5;//will get changed when testing
    private int distance;

    public Shooter(LinearOpMode opMode){
        op = opMode;
        hardwareMap = op.hardwareMap;

        shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");//gets the name ShooterMotor from hardware map and assigns it to shooter_Motor
        shooter_Servo=hardwareMap.servo.get("ShooterServo");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter_Servo.setPosition(1.0);
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


    public void moveServoPosition(double pushPosition) {
        op.telemetry.addData("claw position :", pushPosition);
        op.telemetry.update();
        shooter_Servo.setPosition(pushPosition);
        op.sleep(2000);
    }

    public void shootGoalTeleop(int distance, int power){
        double sleepTime = (distance / speedTopGoal * 1000);

        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooterMotor.setTargetPosition(distance);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setTargetPosition(distance);
        shooterMotor.setPower(power);
        if(shooterMotor.getCurrentPosition()==distance){
            shooterMotor.setPower(0);
        }

    }

    public void shootHighGoal(int rings) {
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(speedTopGoal);
        op.sleep(1000);
        for(int i=0;i<rings;i++){
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);
    }

    public void shootMidGoal(int rings){
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPower(speedMediumGoal);
        //while (shooterMotor.isBusy()) {
        op.sleep(1000);
        for(int i=0;i<rings;i++){
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);
    }

    public void shootLowGoal(int  distance){
        this.distance=distance;
        double sleepTime = (distance / speedLowGoal * 1000);
        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooterMotor.setTargetPosition(distance);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setPower(speedLowGoal);

        while (shooterMotor.isBusy()) {
            op.sleep(1000);
            moveServo(false);
            moveServo(true);
            moveServo(false);
            moveServo(true);
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);
    }


}
