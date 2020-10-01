package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
class motorMode {
    DcMotor motor;
    DcMotor.RunMode mode;
    motorMode(DcMotor motor, DcMotor.RunMode mode){
        this.motor = motor;
        this.mode = mode;
    }
}

public class HardwareMapV2 {
    DcMotor frontRight, frontLeft, backRight, backLeft, intake, outtake;
    DcMotor leftVertical, rightVertical, horizontal;
    ArrayList<DcMotor> motors = new ArrayList<>(Arrays.asList(frontRight, frontLeft, backLeft, backRight, intake, outtake));
    ArrayList<DcMotor> odomotors = new ArrayList<>(Arrays.asList(leftVertical, rightVertical, horizontal));
    ModernRoboticsI2cGyro realgyro1;
    HardwareMap hwMap;


    public void init () {
        frontLeft = hwMap.dcMotor.get("front_left");
        frontRight = hwMap.dcMotor.get("front_right");
        backRight = hwMap.dcMotor.get("back_right");
        backLeft = hwMap.dcMotor.get("back_left");
        intake = hwMap.dcMotor.get("succ");
        outtake = hwMap.dcMotor.get("spit");
        leftVertical = hwMap.dcMotor.get("left_vertical");
        rightVertical = hwMap.dcMotor.get("right_vertical");
        horizontal = hwMap.dcMotor.get("horizontal");


        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setEncoders(ArrayList<DcMotor> motors, DcMotor.RunMode mode){
        for (DcMotor motor: motors){
            motor.setMode(mode);
        }
    }

    public void setEncoders(DcMotor.RunMode mode){
        setEncoders(motors, mode);
        setEncoders(odomotors, mode);
    }

    public void setEncoders(ArrayList<DcMotor> motors, DcMotor.RunMode mode, motorMode... mm){
        for (DcMotor motor: motors){
            for (motorMode m : mm) {
                if (m.equals(motor)) {
                    motor.setMode(m.mode);
                }
                else {
                    motor.setMode(mode);
                }
            }
        }
    }
    public void setPowerAll(double power){
        for (DcMotor motor: motors){
            motor.setPower(power);
        }
    }

}
