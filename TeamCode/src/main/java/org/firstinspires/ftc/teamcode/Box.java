package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Box {
    private final Servo flywheelServo1;
    private final Servo flywheelServo2;
    private final Servo flapServo;
    private boolean boxFull;

    private double flapClosed = 0.5;
    private double flapOpen = -0.5;

    //boolean boxFull has to receive input from break beam sensor


    public Box(OpMode opMode) {
        flywheelServo1 = opMode.hardwareMap.servo.get("flywheel servo");
        flywheelServo2 = opMode.hardwareMap.servo.get("flywheel servo2");
        flapServo = opMode.hardwareMap.servo.get("flap servo");
    }

    public void depositFirstPixel(){
        flywheelServo1.setPosition(0.5);
        flapServo.setPosition(flapOpen);
    }

    public void depositSecondPixel(){
        flywheelServo2.setPosition(0.5);
        flapServo.setPosition(flapOpen);
    }


    public void openFlap(){
        if(Bot.currentState == Bot.BotState.OUTTAKE && boxFull){
            flapServo.setPosition(flapOpen);
        }
    }

    public void resetBox(){
        flapServo.setPosition(flapClosed);
        flywheelServo1.setPosition(0);
        flywheelServo2.setPosition(0);
    }

}
