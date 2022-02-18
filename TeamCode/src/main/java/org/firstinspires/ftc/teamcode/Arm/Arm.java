package org.firstinspires.ftc.teamcode.Arm;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;



public class Arm {



    DcMotorEx winchMotor;


    public static final int startPos = -1000;
    double rotPos = 0.1;
    double rotFinalPos = 0.9;
    double rotMiddlePos = 0.5;

    Servo rotMotor;
    Servo cubeMotor;
    Servo teamElementMotor;
    public Arm(Telemetry telemetry, HardwareMap hardwareMap){
        winchMotor = hardwareMap.get(DcMotorEx.class, "winch");
        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setTargetPosition(0);
        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        winchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10.00, 0.05, 0.0, 0.0, MotorControlAlgorithm.LegacyPID));
        telemetry.addData("PID", winchMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        Servo rotMotor = hardwareMap.get(Servo.class, "rot");
        Servo cubeMotor = hardwareMap.get(Servo.class, "cube");
        Servo teamElementMotor = hardwareMap.get(Servo.class, "teamElement");
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


    public int getWinchPosition(){
        return winchMotor.getCurrentPosition();

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
            winchMotor.setTargetPosition(800-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==2){
            winchMotor.setTargetPosition(1800-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==3){
            winchMotor.setTargetPosition(2800-startPos);
            winchMotor.setPower(.5);
        } else if(lvl==4){
            winchMotor.setTargetPosition(3000-startPos);
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

//    public void rotateLeft(boolean pressed){
//        if(pressed){
//
//        }
//    }

    //Custom position
    public void AutoRotateArm(int position){
            rotMotor.setPosition(position);
        }
    //Initial position of the arm
    public void resetArmPosition(){
        rotMotor.setPosition(rotPos);
    }


    //Increases right position of the arm
    public void armRight(boolean imp){
        if (imp){
            if (rotMotor.getPosition() < rotMiddlePos){
                rotMotor.setPosition(rotPos);
            }
            else{
                rotMotor.setPosition(rotMotor.getPosition() - 0.4);
            }
        }
    }


    //Increases left position of the arm
    public void armLeft(boolean imp){
        if (imp){
            if (rotMotor.getPosition() > rotMiddlePos){
                rotMotor.setPosition(rotFinalPos);
            }
            else{
                rotMotor.setPosition(rotMotor.getPosition() + 0.4);
            }
        }
    }



    //Release the cube
    public void cubeMotorRelease(){
        cubeMotor.setPosition(0);
    }

    //Blocks the cube
    public void cubeMotorReset(){
        cubeMotor.setPosition(0.5);
    }



    //Releases(Resets) the arm
    public void resetArm(boolean inp){
        if (inp){
            teamElementMotor.setPosition(0.4);
        }
    }

    //Grabs the element
    public void grabArm(boolean inp){
        if (inp){
            teamElementMotor.setPosition(0.15);
        }
    }
}


