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

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        motor = hwMap.get(DcMotor.class, "swingArm");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void swingUp(){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        speed = 1;
        Position = 75;
        holdingPower = 1;

        motor.setTargetPosition(Position);
        //Set the power of the motor.
        motor.setPower(speed);
        //set the mode to go to the target position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setPower(holdingPower);

    }

    public void swingDown(){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        speed = 0.5;
        Position = 20;
        holdingPower = 0;

        motor.setTargetPosition(Position);
        //Set the power of the motor.
        motor.setPower(speed);
        //set the mode to go to the target position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setPower(holdingPower);

    }
}
