package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AxonAbsolutePositionEncoder;

public class Wrist {

    public enum WristStates{
        SampleDepositMode,
        SampleIntakeMode,
        ParallelMode,
    }


    public final double LOWER_LIMIT = 0.53; // 0.25 = 90 degrees
    public final double UPPER_LIMIT = 1;
    public final double WRIST_PARALLEL = 0.78; // Servo position to be parallel when Arm Rotation = 0
    private double SAMPLE_DEPOSIT_MODE_POSITION = WRIST_PARALLEL + 0.03; //0.03 offset relative to Parallel
    private double SAMPLE_INTAKE_MODE_POSITION = WRIST_PARALLEL - 0.1;// -0.1 offset relative to Parallel

    private double callsPerSecond = 50;
    private double rotationPerSecond = 0.2 / callsPerSecond; //Max of 0.04 change in servo position. Up to 100 calls per second
    private long lastCalled = 0;
    private WristStates currentState;

    private Servo wrist;

    //------------------------------------------------------------------------------------------
    //----------------------------------Initialization Constructors-----------------------------
    //------------------------------------------------------------------------------------------
    public Wrist(HardwareMap hw){
        this(hw, "wrist");
    }

    public Wrist(HardwareMap hw, String name){
        wrist = hw.get(Servo.class, name);
        wrist.setDirection(Servo.Direction.REVERSE);
        currentState = WristStates.SampleIntakeMode;

    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Set Position Functions----------------------------------
    //------------------------------------------------------------------------------------------

    public void setParallelMode(){
        currentState = WristStates.ParallelMode;
    }

    public void setSampleDepositMode(){
        currentState = WristStates.SampleDepositMode;
        wrist.setPosition(SAMPLE_DEPOSIT_MODE_POSITION);
    }

    public void wristParallelToGround(double angleToGround){
        if (currentState == WristStates.ParallelMode){
            double newPos = clamp(WRIST_PARALLEL -  (angleToGround / 360), LOWER_LIMIT, UPPER_LIMIT);
            wrist.setPosition(newPos);
        }
    }

    public void setSampleIntakeMode(){
        currentState = WristStates.SampleIntakeMode;
        wrist.setPosition(SAMPLE_INTAKE_MODE_POSITION);
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    public double getWristPosition(){
        return wrist.getPosition();
    }

    public WristStates getState(){
        return currentState;
    }

    //------------------------------------------------------------------------------------------
    //----------------------------Tuning/Tweaking Functions-------------------------------------
    //------------------------------------------------------------------------------------------
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

    //------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    private double clamp(double value, double min, double max){
        return Math.max( min , Math.min( max , value));
    }
}
