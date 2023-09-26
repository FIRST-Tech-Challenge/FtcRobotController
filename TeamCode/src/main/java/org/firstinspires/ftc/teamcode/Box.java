package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Box {
    private final Servo flywheelServo;
    private final Servo flapServo;
    private boolean boxFull;
    private boolean flapClosed;
    private double flapOpenOrClosedPos = 0.5;
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
                return;
            }else{
                flywheelServo.setPosition(grabPixelPos);
                flapServo.setPosition(flapOpenOrClosedPos);
                this.boxFull = true;
            }
        }
    }

    //INCOMPLETE

}
