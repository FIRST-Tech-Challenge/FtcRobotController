package org.firstinspires.ftc.teamcode.Arm;



import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorEx;





public class Arm {



    DcMotorEx winchMotor;


    public static final int startPos = 800;
    int rotPos = 0;

    //Servo rotMotor;
    //Servo cubeMotor;
    //Servo teamElementMotor;
    public Arm(HardwareMap hardwareMap){
        winchMotor = hardwareMap.get(DcMotorEx.class, "winch");
        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setTargetPosition(0);
        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //Servo rotMotor = hardwareMap.get(Servo.class, "rot");
        //Servo cubeMotor = hardwareMap.get(Servo.class, "cube");
        //Servo teamElementMotor = hardwareMap.get(Servo.class, "teamElement");
    }

    boolean prevPress = false;
    int lvl = 0;


    //Autonomous

    public void setWinchPosition(int winchPosition){
        winchMotor.setTargetPosition(winchPosition - startPos);
        winchMotor.setPower(.5);
    }


    public void resetWinchPosition(){
        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }





    public void moveWinchUp(boolean pressed){
        if(pressed && !prevPress && lvl < 3){
            lvl++;
        }
        prevPress = pressed;
        if(lvl==0){
            winchMotor.setTargetPosition(0-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==1){
            winchMotor.setTargetPosition(1000-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==2){
            winchMotor.setTargetPosition(2000-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==3){
            winchMotor.setTargetPosition(3000-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==4){
            winchMotor.setTargetPosition(3500-startPos);
            winchMotor.setPower(.5);
        }
    }



    boolean prevPress2 = false;
    public void moveWinchDown(boolean pressed){
        if(pressed && !prevPress2 && lvl > 0){
            lvl--;
        }
        prevPress2 = pressed;
    }

    public void rotateLeft(boolean pressed){
        if(pressed){

        }
    }


}