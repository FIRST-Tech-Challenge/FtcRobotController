package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    enum WristStates{
        IntakeMode,
        HoverMode,
        DepositMode
    }

    private double INTAKE_MODE_POSITION = 0.1;
    private double HOVER_MODE_POSITION = 0.15;
    private double DEPOSIT_MODE_POSITION = 0.23;

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
    public void setPosition(double pos){
        wrist.setPosition(pos);
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

    public WristStates getState(){
        return currentState;
    }

}
