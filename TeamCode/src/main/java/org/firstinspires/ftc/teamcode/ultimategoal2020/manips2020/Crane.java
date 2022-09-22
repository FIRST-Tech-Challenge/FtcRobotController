package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Crane implements EbotsManip2020 {

    private DcMotorEx craneMotor;
    final int CRANE_OVER_WALL_HEIGHT = 222;
    final int CRANE_MIN_CRANE_HEIGHT = 334;
    final int CRANE_DRAG_HEIGHT = 290;
    final int CRANE_VERTICAL_HEIGHT = 145;
    int CRANE_ENCODER_OFFSET = 0;
    boolean debugOn = true;
    String logTag = "EBOTS";


    public Crane(HardwareMap hardwareMap){
        if (debugOn){
            Log.d(logTag, "Instantiating Crane...");
        }
        craneMotor = hardwareMap.get(DcMotorEx.class, "crane");
        craneMotor.setDirection(DcMotorEx.Direction.FORWARD);
        craneMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Deprecated
    public int getCurrentPosition(){
        return craneMotor.getCurrentPosition();
    }

    @Deprecated
    public void setPower(double powerLevel){
        craneMotor.setPower(powerLevel);
    }

    public double getPower(){
        return craneMotor.getPower();
    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        if (debugOn){
            Log.d(logTag, "Crane::handleGamepadInput...");
        }


        int cranePos = craneMotor.getCurrentPosition() + CRANE_ENCODER_OFFSET;
        boolean dragWobble = false;
        boolean liftOverWall = false;
        boolean goVertical = false;

        // get the controller input for crane
        // set boolean values
        double craneInput = 0;
        if(gamepad.dpad_up) {
            craneInput = 1;
            liftOverWall = true;
        } else if(gamepad.dpad_down) {
            craneInput = -1;
        } else if(gamepad.dpad_right){
            craneInput = 1;
            goVertical=true;
        } else if (gamepad.dpad_left){
            craneInput = 1;
            dragWobble = true;
        }

        // Set the max allowable height
        int MAX_HEIGHT = CRANE_VERTICAL_HEIGHT;
        if(dragWobble){
            MAX_HEIGHT = CRANE_DRAG_HEIGHT;
        } else if(liftOverWall){
            MAX_HEIGHT = CRANE_OVER_WALL_HEIGHT;
        } else if(goVertical){
            MAX_HEIGHT = CRANE_VERTICAL_HEIGHT;
        }

        boolean allowUpwardsTravel = cranePos > MAX_HEIGHT;        //only allow upwards travel if greater than max height
        boolean requestingUpwardsTravel = (Math.signum(craneInput) == 1);
        double passPower = 0;
        //  UPWARD Travel

        if (craneInput==0){
            passPower = 0;
        }else if(requestingUpwardsTravel && allowUpwardsTravel) {
            if (dragWobble && cranePos < (MAX_HEIGHT + 5)) {
                passPower = -0.5;
            }else if (liftOverWall && cranePos < (MAX_HEIGHT + 5)) {
                passPower = -0.8;
            }else if(goVertical && cranePos < (MAX_HEIGHT + 5)){
                passPower = -0.8;
            }else {
                passPower = -0.8;
            }
        }
        // if requesting downward travel, and want to go slow at end
        else if(!requestingUpwardsTravel){
            boolean allowDownwardTravel = cranePos < CRANE_MIN_CRANE_HEIGHT;

            if (!allowUpwardsTravel) passPower = 0.8;  //Apply high power while unfolding
            else if (!allowDownwardTravel)
                passPower = 0;        //No power after encoder hits 160;
            else passPower = 0.2;       //Lower power if close to bottom
        }

        craneMotor.setPower(passPower);
//        String f = "%.2f";
//        Log.d(logTag, "---------------------------------");
//        Log.d(logTag, "dragWobble / liftOverWall: " + dragWobble + " / " + liftOverWall);
//        Log.d(logTag, "Crane Pos: " + cranePos);
//        Log.d(logTag, "Crane passPower: " + String.format(f, passPower));
//        Log.d(logTag, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");


        if(gamepad.left_bumper && gamepad.right_bumper & gamepad.right_stick_button){
            resetCraneEncoder();
            CRANE_ENCODER_OFFSET = CRANE_MIN_CRANE_HEIGHT;
        }

    }

    @Override
    public void stop() {
        craneMotor.setPower(0);
    }


    public void resetCraneEncoder(){
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public int unfoldCrane(){
        // Actuates motors to unfolds the crane and returns the current position
        int cranePos = craneMotor.getCurrentPosition();
        if (cranePos < CRANE_VERTICAL_HEIGHT) {
            // Need max power when crane is fully folded
            craneMotor.setPower(1);
        } else if (cranePos < CRANE_DRAG_HEIGHT){
            // Between vertical and drag height slow down
            craneMotor.setPower(0.45);
        } else {
            // After drag height, go slow until bottom is hit
            craneMotor.setPower(0.25);
        }
        return cranePos;
    }


    public int moveCraneToDragWobbleGoal() {
        int cranePos = craneMotor.getCurrentPosition();
        int MAX_HEIGHT = CRANE_DRAG_HEIGHT;
        boolean allowUpwardsTravel = cranePos > MAX_HEIGHT;        //only allow upwards travel if greater than max height
        double passPower = 0;
        if (allowUpwardsTravel) passPower = (cranePos < (MAX_HEIGHT + 5)) ? -0.4 : -1.0;
        craneMotor.setPower(passPower);
        return cranePos;
    }

}
