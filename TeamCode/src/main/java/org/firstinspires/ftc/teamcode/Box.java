package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Box {
    private final Servo flywheelServo;
    private final Servo flapServo;
    private boolean pixelsCollected;
    private boolean boxFull;
    private double flapClosed = 0.5;
    private double flapOpen = -0.5;
    private double grabPixelPos = 0.5;


    public Box(OpMode opMode) {
        flywheelServo = opMode.hardwareMap.servo.get("flywheel servo");
        flapServo = opMode.hardwareMap.servo.get("flap servo");

        flywheelServo.setDirection(Servo.Direction.FORWARD);
        flapServo.setDirection(Servo.Direction.FORWARD);
    }

    public void collectPixel(){
        if(Bot.currentState == Bot.BotState.OUTTAKE){
            if(boxFull){
                pixelsCollected = true;
                return;
            }else{
                flapServo.setPosition(flapClosed);
                flywheelServo.setPosition(grabPixelPos);
                this.boxFull = true;
                pixelsCollected = true;
            }
        }
    }

    public void outtakeBox(){
        if(Bot.currentState == Bot.BotState.OUTTAKE && pixelsCollected){
            flapServo.setPosition(flapOpen);
        }
    }

    public void resetBox(){
        flapServo.setPosition(flapClosed);
        flywheelServo.setPosition(0);
        boxFull = false;
    }

}
