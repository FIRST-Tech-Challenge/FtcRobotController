package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


public class Box {
    private final Servo wheelServo;
    private final Servo flapServo;
    private final Servo angleServo;
    private int numPixelsDeposited;
    private final DigitalChannel breakbeamSensor;
    private boolean boxFull;

    private int timesBroken;
    private final double flapClosed = 1;
    private final double flapOpen = 0;
    private final double anglePosition=0.5;

    //boolean boxFull has to receive input from break beam sensor


    public Box(OpMode opMode) {
        wheelServo = opMode.hardwareMap.servo.get("wheel servo");
        flapServo = opMode.hardwareMap.servo.get("flap servo");
        angleServo = opMode.hardwareMap.servo.get("angle servo");
        breakbeamSensor = hardwareMap.get(DigitalChannel.class, "breakbeamSensor");
        breakbeamSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void depositFirstPixel(){
        //purely gravity, open the flap
        angleServo.setPosition(anglePosition);
        flapServo.setPosition(flapOpen);
        runWheel(10);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel(){
        angleServo.setPosition(anglePosition);
        flapServo.setPosition(flapOpen);
        runWheel(1);
        numPixelsDeposited = 2;
    }
    public void adjustAngle(double position){
        angleServo.setPosition(position);
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
        angleServo.setPosition(0);
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
           setIsFull(true);
        }
    }

    public int getNumPixelsDeposited(){
        return numPixelsDeposited;
    }

}
