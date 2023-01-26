package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {

    HardwareMap hwMap = null;
    public DcMotor motor;
    int timeout_ms =  5000;
    double speed;
    int Position;
    double holdingPower;

    private int YEAR = 2021;


    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        motor = hwMap.get(DcMotor.class, "swingArm");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void swingUp(){
        ElapsedTime runtime = new ElapsedTime();
        speed = 0.7;
        Position = 495;
        timeout_ms = 500;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(Position);
        //set the mode to go to the target position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set the power of the motor.
        motor.setPower(speed);

        runtime.reset();

        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);

    }

    public void swingDown(){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        speed = 0.7;
        Position = 55;

        motor.setTargetPosition(Position);
        //set the mode to go to the target position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set the power of the motor.
        motor.setPower(speed);


        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }

    }
}
