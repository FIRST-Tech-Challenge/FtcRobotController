package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AxonAbsolutePositionEncoder;

public class Wrist {

    public enum WristStates{
        IntakeMode,
        HoverMode,
        DepositMode,
        SampleIntakeMode
    }

    private double INTAKE_MODE_POSITION = 0.49;
    private double HOVER_MODE_POSITION = 0.7;
    private double DEPOSIT_MODE_POSITION = 0.6;

    private double SAMPLE_INTAKE_MODE_POSITION = 0.54;

    private double callsPerSecond = 50;
    private double rotationPerSecond = 0.2 / callsPerSecond; //Max of 0.04 change in servo position. Up to 100 calls per second
    private long lastCalled = 0;
    private WristStates currentState;

    private Servo wrist;

    public Wrist(HardwareMap hw){
        this(hw, "wrist");
    }

    public Wrist(HardwareMap hw, String name){
        wrist = hw.get(Servo.class, name);
        wrist.setDirection(Servo.Direction.REVERSE);
        currentState = WristStates.IntakeMode;

    }

    // For manual testing
    public void setPosition(double power){
        long timeNow = System.currentTimeMillis();
        if (timeNow < (lastCalled + (1000/callsPerSecond))){
            return;
        }
        lastCalled = timeNow;
        double newPos = wrist.getPosition() + (power * rotationPerSecond);
        wrist.setPosition(newPos);
    }

    public void setIntakeMode(){
        currentState = WristStates.IntakeMode;
        wrist.setPosition(INTAKE_MODE_POSITION);
    }

    public void setHoverMode(){
        currentState = WristStates.HoverMode;
        wrist.setPosition(HOVER_MODE_POSITION);
    }

    public void setDepositMode(){
        currentState = WristStates.DepositMode;
        wrist.setPosition(DEPOSIT_MODE_POSITION);
    }

    public void setSampleIntakeMode(){
        currentState = WristStates.SampleIntakeMode;
        wrist.setPosition(SAMPLE_INTAKE_MODE_POSITION);
    }

    public double getWristPosition(){
        return wrist.getPosition();
    }

    public WristStates getState(){
        return currentState;
    }

}
