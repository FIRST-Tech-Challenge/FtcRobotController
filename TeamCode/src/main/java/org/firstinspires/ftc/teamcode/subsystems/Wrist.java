package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AxonAbsolutePositionEncoder;

import java.util.HashMap;

public class Wrist {

    public enum WristStates{
        SampleDepositMode,
        SampleIntakeMode,
        ParallelMode,
    }

    private HashMap<WristStates, Double> wristPositions;

    public final double LOWER_LIMIT = 0.53; // 0.25 = 90 degrees
    public final double UPPER_LIMIT = 1;
    public final double WRIST_PARALLEL = 0.78; // Servo position to be parallel when Arm Rotation = 0

    private double lastAngle;
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

        wristPositions = new HashMap<WristStates, Double>();
        wristPositions.put(WristStates.ParallelMode, WRIST_PARALLEL);
        wristPositions.put(WristStates.SampleDepositMode, WRIST_PARALLEL + 0.3);
        wristPositions.put(WristStates.SampleIntakeMode, WRIST_PARALLEL - 0.1);
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Set Position Functions----------------------------------
    //------------------------------------------------------------------------------------------


    public void wristParallelToGround(double angleToGround){
        lastAngle = angleToGround;
        if (currentState == WristStates.ParallelMode){
            double newPos = clamp(WRIST_PARALLEL -  (angleToGround / 360), LOWER_LIMIT, UPPER_LIMIT);
            wrist.setPosition(newPos);
        }
    }
    public void goToPosition(WristStates state){
        currentState = state;
        if (state != WristStates.ParallelMode) {
            wrist.setPosition(wristPositions.get(state));
        }
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
        if (currentState == WristStates.ParallelMode){
            wristPositions.put(currentState, (power * rotationPerSecond) +
                    clamp(WRIST_PARALLEL -  (lastAngle / 360), LOWER_LIMIT, UPPER_LIMIT));
        }
        else{
            wristPositions.put(currentState, newPos);
        }

        wrist.setPosition(newPos);
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    private double clamp(double value, double min, double max){
        return Math.max( min , Math.min( max , value));
    }
}
