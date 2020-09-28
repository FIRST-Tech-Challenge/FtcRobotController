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
    ArrayList<DcMotor> motors = new ArrayList<>(Arrays.asList(frontRight, frontLeft, backLeft, backRight, intake, outtake));
    ModernRoboticsI2cGyro realgyro1;
    HardwareMap hwMap;


    public void init () {
        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        intake = hwMap.get(DcMotor.class, "succ");
        outtake = hwMap.get(DcMotor.class, "spit");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setEncoders(DcMotor.RunMode mode){
        for (DcMotor motor: motors){
            motor.setMode(mode);
        }
    }
    public void setEncoders(DcMotor.RunMode mode, motorMode... mm){
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
