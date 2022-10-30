package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    //8192 * 5.23 * 1.8 * 4
    public final double     DEFAULT_TURRET_POWER = 0.1;
    public final double     INITIAL_MOVE_LEFT_TURRET_POWER = 0.1;
    public final double     HARD_STOP_CURRENT_DRAW = 100;
    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final String     TRANSITION_STATE = "TRANSITION_STATE";
    public final int        LEFT_POSITION_VALUE = 3;
    public final int        CENTER_POSITION_VALUE = 264;
    public final int        RIGHT_POSITION_VALUE = 525;
    public final int        ANGLE_TOLERANCE = 5;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 75;

    public Telemetry telemetry;
    public DcMotorEx turretMotor;

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        //getting turret motor from the hardware map
        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setState(String desiredState, Lift lift){
        String currentState = getCurrentState();
        telemetry.addData("turretCurrentState" , currentState);
        telemetry.addData("turretDesiredState" , currentState);
        telemetry.addData("turretMotorPosition", turretMotor.getCurrentPosition());
        if(isLiftTooLow(lift) || desiredState.equalsIgnoreCase(currentState)){
            turretMotor.setPower(0);
            return;
        }
        else{
            selectTransition(desiredState, currentState);
        }
    }

    public boolean isLiftTooLow(Lift lift) {
        boolean tooLow = lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
        telemetry.addData("liftTooLow" , tooLow);
        return tooLow;
    }

    private void selectTransition(String desiredLevel, String currentState){
        switch(desiredLevel){
            case LEFT_POSITION:{
                transitionToPosition(LEFT_POSITION_VALUE);
                break;
            } case CENTER_POSITION:{
                transitionToPosition(CENTER_POSITION_VALUE);
                break;
            } case RIGHT_POSITION:{
                transitionToPosition(RIGHT_POSITION_VALUE);
                break;
            }
        }
    }

    private void transitionToPosition(int ticks){
        moveTo(ticks);
    }

    public void moveTo (int positionInTicks) {
        //raising heights to reach different junctions, so four values
        telemetry.addData("turnLiftCalled" , true);
        turretMotor.setTargetPosition(positionInTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
    }

    public String getCurrentState() {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        telemetry.addData("CurrentMotorEncoderTicks", turretMotor.getCurrentPosition());
        telemetry.addData("CurrentPosition", currentPosition);
        if(inTolerance(currentPosition, LEFT_POSITION_VALUE)){
            state = LEFT_POSITION;
        } else if (inTolerance(currentPosition, CENTER_POSITION_VALUE)) {
            state = CENTER_POSITION;
        } else if (inTolerance(currentPosition, RIGHT_POSITION_VALUE)) {
            state = RIGHT_POSITION;
        }
        return state;
    }

    private boolean inTolerance(double actualTicks, double desiredTicks) {
        return (actualTicks > desiredTicks - ANGLE_TOLERANCE) && (actualTicks < desiredTicks + ANGLE_TOLERANCE);
    }

    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization

        while(turretMotor.getCurrent(CurrentUnit.MILLIAMPS) < HARD_STOP_CURRENT_DRAW) {
            turretMotor.setPower(INITIAL_MOVE_LEFT_TURRET_POWER);
        }
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }
}
