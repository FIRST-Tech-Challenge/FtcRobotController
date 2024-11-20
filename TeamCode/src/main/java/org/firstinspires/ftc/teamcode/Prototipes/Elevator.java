package org.firstinspires.ftc.teamcode.Prototipes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator {

    final public static ARM_MAX_LIMIT = 20000;
    final public static ARM_MAX_LIMIT = 200;

    private DcMotor elevatorExtend;
    private DcMotor elevatorArm;

    private OpMode opMode;

    public Elevator(OpMode opMode) {
        this.opMode = opMode;
    }

    public void initElevator(){
        elevatorExtend  = opMode.hardwareMap.get(DcMotor.class, "elevatorExtend");
        elevatorArm = opMode.hardwareMap.get(DcMotor.class, "elevatorArm");
        elevatorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // this function use value (like the gamepad stick) to give the extension motor power.
    public void extend(double extension){
        if (elevatorExtend.getPower()>=0.3){
            elevatorExtend.setPower(extension);
        }
    }
    // this function use value (like the gamepad stick) to give the rotation motor power.
    public void rotateForword(){
//        elevatorArm.setPower(rotation);
        if(elevatorArm.getCurrentPosition()>=ARM_MAX_LIMIT){
            elevatorArm.setTargetPosition(elevatorArm.getCurrentPosition());
        }
        else {elevatorArm.setTargetPosition(elevatorArm.getCurrentPosition()+200);} // todo check real limit numbers

    }
    public void rotateBackword(){
//        elevatorArm.setPower(rotation);
        if(elevatorArm.getCurrentPosition()<=ARM_MIN_LIMIT){
            elevatorArm.setTargetPosition(elevatorArm.getCurrentPosition());
        }
        else {elevatorArm.setTargetPosition(elevatorArm.getCurrentPosition()-200);} // todo check real limit numbers
    }

}

