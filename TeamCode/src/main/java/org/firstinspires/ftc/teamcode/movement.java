package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.HardwareMap;

public class movement {

    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public DcMotor Motor4;

    public movement(ArrayList<DcMotor> motors) {
        Motor1 = motors.get(0);
        Motor2 = motors.get(1);
        Motor3 = motors.get(2);
        Motor4 = motors.get(3);
    }

    //stop movement
    public void stop(){
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
    }

    //vertical movement
    public void verticalMovement(float power){
        Motor1.setPower(power * 1);
        Motor2.setPower(power * 1);
        Motor3.setPower(power * -1);
        Motor4.setPower(power * -1);
    }
    //horizontal movement
    public void horizontalMovement(float power){
        Motor1.setPower(power * -1);
        Motor2.setPower(power * 1);
        Motor3.setPower(power * -1);
        Motor4.setPower(power * 1);
    }
    //counter-clockwise rotation
    public void rotateMovement(float power){
        power = (float) (power * 0.5);
        Motor1.setPower(power);
        Motor2.setPower(power);
        Motor3.setPower(power);
        Motor4.setPower(power);
    }
    /*
    public void telmotor(){
        telemetry.addData("Motor1", Motor1);
        telemetry.addData("Motor2", Motor2);
        telemetry.addData("Motor3", Motor3);
        telemetry.addData("Motor4", Motor4);
        telemetry.update();
    }
    */
}
