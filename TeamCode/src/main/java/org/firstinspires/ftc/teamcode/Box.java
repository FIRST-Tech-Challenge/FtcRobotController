package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Box {
    private final Servo wheelServo;
    private final Servo flapServo;
    private int numPixelsDeposited;
    private DigitalChannel breakbeamSensor;
    private boolean boxFull;

    private int timesBroken;
    private double flapClosed = 1;
    private double flapOpen = 0;

    //boolean boxFull has to receive input from break beam sensor


    public Box(OpMode opMode) {
        wheelServo = opMode.hardwareMap.servo.get("wheel servo");
        flapServo = opMode.hardwareMap.servo.get("flap servo");
        breakbeamSensor = hardwareMap.get(DigitalChannel.class, "breakbeamSensor");
        breakbeamSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void depositFirstPixel(){
        //purely gravity, open the flap
        flapServo.setPosition(flapOpen);
        runWheel(10);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel(){
        flapServo.setPosition(flapOpen);
        runWheel(1);
        numPixelsDeposited = 2;
    }


    public void runWheel(double inc){
        for(int i = 1; i<=inc; i++){
            wheelServo.setPosition(i* (1/inc));
        }
    }
    //wheel spins in reverse to keep the pixel from falling out
// 2 servos - one for wheel, one for flap
    public void resetBox(){
        timesBroken= 0;
        numPixelsDeposited = 0;
        flapServo.setPosition(flapClosed);
        wheelServo.setPosition(0);
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
