package org.firstinspires.ftc.teamcode.Arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Arm {

    DcMotorEx winch;
    Servo rotMotor;
    Servo cubeMotor;
    Servo teamElementMotor;
    public Arm(HardwareMap hardwareMap){
        DcMotorEx winch = hardwareMap.get(DcMotorEx.class, "winch");
        winch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Servo rotMotor = hardwareMap.get(Servo.class, "rot");
        Servo cubeMotor = hardwareMap.get(Servo.class, "rot");
        Servo teamElementMotor = hardwareMap.get(Servo.class, "rot");
    }

    boolean prevPress = false;
    int lvl = 0;
    public void moveWinchUp(boolean pressed){
        if(pressed && !prevPress && lvl < 3){
            lvl++;
        }
        if(lvl==0){
            winch.setTargetPosition(0);
            winch.setPower(.5);
        } else if(lvl==1){
            winch.setTargetPosition(200);
            winch.setPower(.5);
        } else if(lvl==2){
            winch.setTargetPosition(400);
            winch.setPower(.5);
        } else if(lvl==3){
            winch.setTargetPosition(600);
            winch.setPower(.5);
        }
    }

    boolean prevPress2 = false;
    public void moveWinchDown(boolean pressed){
        if(pressed && !prevPress2 && lvl > 0){
            lvl--;
        }
    }

}
