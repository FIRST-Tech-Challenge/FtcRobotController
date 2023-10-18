package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Box {
    private final Servo flywheelServo1;
    private final Servo flywheelServo2;
    private final Servo flapServo;
    private int numPixelsDeposited;
    private DigitalChannel breakbeamSensor;
    private boolean boxFull;

    private int timesBroken;
    private double flapClosed = 0.5;
    private double flapOpen = -0.5;

    //boolean boxFull has to receive input from break beam sensor


    public Box(OpMode opMode) {
        flywheelServo1 = opMode.hardwareMap.servo.get("flywheel servo");
        flywheelServo2 = opMode.hardwareMap.servo.get("flywheel servo2");
        flapServo = opMode.hardwareMap.servo.get("flap servo");
        breakbeamSensor = hardwareMap.get(DigitalChannel.class, "breakbeamSensor");
        breakbeamSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void depositFirstPixel(){
        flywheelServo1.setPosition(0.5);
        flapServo.setPosition(flapOpen);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel(){
        flywheelServo2.setPosition(0.5);
        flapServo.setPosition(flapOpen);
        numPixelsDeposited = 2;
    }


    public void openFlap(){
        if(Bot.currentState == Bot.BotState.OUTTAKE && boxFull){
            flapServo.setPosition(flapOpen);
        }
    }



    public void resetBox(){
        timesBroken= 0;
        numPixelsDeposited = 0;
        flapServo.setPosition(flapClosed);
        flywheelServo1.setPosition(0);
        flywheelServo2.setPosition(0);
    }

    public boolean getIsFull(){
        return boxFull;
    }

    public void setIsFull(boolean isFull){
        boxFull = isFull;
    }

    public void checkBeam(){
        boolean isBeamBroken = breakbeamSensor.getState();
        if (isBeamBroken) {
            telemetry.addData("Status", "Object detected!");
            timesBroken++;
        } else {
            telemetry.addData("Status", "No object detected");
        }
        if(timesBroken ==2){
            Bot.box.setIsFull(true);
        }
    }

    public int getNumPixelsDeposited(){
        return numPixelsDeposited;
    }

}
