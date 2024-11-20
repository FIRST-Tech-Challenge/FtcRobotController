package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

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
        SubHoverMode,

        PerpendicularMode,

        SpeciminDepositMode
    }

    private HashMap<WristStates, Double> wristPositions;

    public final double LOWER_LIMIT = 0; // 0.25 = 90 degrees
    public final double UPPER_LIMIT = 0.55;
    public final double WRIST_PARALLEL = 0.218; // Servo position to be parallel when Arm Rotation = 0

    private double lastAngle;
    private double callsPerSecond = 50;
    private double rotationPerSecond = 0.2 / callsPerSecond; //Max of 0.04 change in servo position. Up to 100 calls per second
    private long lastCalled = 0;
    private WristStates currentState;
    private double currentPosition;

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
        wristPositions.put(WristStates.ParallelMode, WRIST_PARALLEL );
        wristPositions.put(WristStates.SubHoverMode, WRIST_PARALLEL + 0.12);
        wristPositions.put(WristStates.SampleDepositMode, WRIST_PARALLEL + -0.152);
        wristPositions.put(WristStates.SampleIntakeMode, WRIST_PARALLEL - 0.015); //0.694
        wristPositions.put(WristStates.PerpendicularMode, WRIST_PARALLEL - 0.25); //Set 90 degrees off from parallel
        wristPositions.put(WristStates.SpeciminDepositMode, WRIST_PARALLEL + 0.02);//0.32
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Set Position Functions----------------------------------
    //------------------------------------------------------------------------------------------


    public void wristParallelToGround(double angleToGround){
        lastAngle = angleToGround;
        if (currentState == WristStates.ParallelMode){
            double newPos = clamp(wristPositions.get(WristStates.ParallelMode) -  (angleToGround / 360), LOWER_LIMIT, UPPER_LIMIT);
//            double newPos = wristPositions.get(WristStates.ParallelMode) -  (angleToGround / 360.0);
            currentPosition = wristPositions.get(WristStates.ParallelMode);
            wrist.setPosition(newPos);
        }
//        else if(currentState == WristStates.PerpendicularMode){
//            double newPos = clamp(wristPositions.get(WristStates.PerpendicularMode) -  ((180-angleToGround) / 360.0), LOWER_LIMIT, UPPER_LIMIT);
//            currentPosition = wristPositions.get(WristStates.PerpendicularMode) -  ((180-angleToGround) / 360.0);
//            wrist.setPosition(newPos);
//        }
    }
    public void goToPosition(WristStates state){
        currentState = state;
        if (state != WristStates.ParallelMode &&
            state != WristStates.PerpendicularMode) {
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
        double newPos = wristPositions.get(currentState) + (power * rotationPerSecond);
//        if (currentState == WristStates.ParallelMode){
//            wristPositions.put(currentState, (power * rotationPerSecond) +
//                    clamp(WRIST_PARALLEL -  (lastAngle / 360), LOWER_LIMIT, UPPER_LIMIT));
//        }
//        else{
        wristPositions.put(currentState, newPos);

        if (currentState != WristStates.ParallelMode /*&&
                currentState != WristStates.PerpendicularMode*/) {
            wrist.setPosition(newPos);
        }
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    private double clamp(double value, double min, double max){
        return Math.max( min , Math.min( max , value));
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format(
                "Wrist Position: %f\n" +
                "Wrist State: %s\n" +
                "Wrist current position: %f\n",
                getWristPosition(),
                currentState,
                currentPosition

        );
    }
}
