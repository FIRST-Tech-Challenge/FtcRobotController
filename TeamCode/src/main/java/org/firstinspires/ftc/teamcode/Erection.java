package org.firstinspires.ftc.teamcode;  //place where the code is located


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Erection{
    private Servo leftServo;
    private Servo rightServo;

    private DcMotorEx frontElevatorEx;
    private DcMotorEx backElevatorEx;

    private DcMotor frontElevator; //DcMotor classes are used as a redundancy so that when the enoders dont work the erection would still work.
    private DcMotor backElevator;
    
    private boolean isError= false;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    
    public void initErection(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        try{
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        frontElevatorEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_EH");
        backElevatorEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_EH");

        frontElevatorEx.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backElevatorEx.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontElevatorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backElevatorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } catch(Exception e){
            isError=true;
        }
    }

    public void raise(double joysitck, double rightStick, boolean height80, boolean height100, boolean height120){
        if (!isError){
        if (height80){
            runToHeight(1184);
        } 
        if (height100){
            runToHeight(1480);
        } 
        if (height120){
            runToHeight(1776);
        }
        if (!(height80 || height100 || height120)){
            frontElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backElevatorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontElevatorEx.setPower(joysitck + 0.5*rightStick);
            backElevatorEx.setPower(-joysitck - 0.5*rightStick);
        }


        telemetry.addData("motor position", backElevatorEx.getCurrentPosition());
        }else{
            
            telemetry.addData("erectile disfunction", isError);
        }
    }
    public void runToHeight(int height){
            frontElevatorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backElevatorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontElevatorEx.setTargetPosition(height);//1000(height mm)/(6mm(hex shaft diameter)*3,14)*28(ticks per rotation)
            backElevatorEx.setTargetPosition(height);
            backElevatorEx.setPower(1);
            frontElevatorEx.setPower(1);
    }

}  