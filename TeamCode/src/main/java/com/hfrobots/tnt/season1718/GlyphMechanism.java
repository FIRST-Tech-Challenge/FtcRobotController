/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1718;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class GlyphMechanism {
    private Servo upperGripper;
    private Servo lowerGripper;
    private final Servo rotate;
    private final Servo gripper1;
    private final Servo gripper2;
    private final DigitalChannel invertedLimitSwitch;
    private final DigitalChannel uprightLimitSwitch;
    protected final Lift lift;

    protected int liftStartPosition;

    private static final double GRIPPER_OPEN = 0.330;
    private static final double GRIPPER_CLOSED = .10;


    public GlyphMechanism(Servo naturalTopGripper, Servo naturalBottomGripper, Servo rotateServo,
                          DigitalChannel invertedLimitSwitch, DigitalChannel uprightLimitSwitch,
                          DigitalChannel minHeight, DigitalChannel maxHeight,
                          DcMotor liftMotor) {
        gripper1 = naturalTopGripper;
        gripper2 = naturalBottomGripper;
        upperGripper = gripper1;
        lowerGripper = gripper2;
        rotate = rotateServo;
        this.invertedLimitSwitch = invertedLimitSwitch;
        this.uprightLimitSwitch = uprightLimitSwitch;
        lift = new Lift();
        lift.minHeight = minHeight;
        lift.maxHeight = maxHeight;
        lift.liftMotor = liftMotor;
        lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftStartPosition = lift.liftMotor.getCurrentPosition();

        upperOpen();
        lowerOpen();
    }

    /* @robot start: upperGripper is gripper1; lowerGripper is gripper2; */
    public void flip(boolean inverted) {
        lowerClose();
        upperClose();
        // don't allow flip unless move 3in up

        //if (Math.abs(lift.liftMotor.getCurrentPosition() - liftStartPosition) < 600) {
        //    Log.d(Constants.LOG_TAG, "Not safe to rotate");

        //     return isFlipped;
        //}

        if (inverted) {
            if (rotate != null) {
                rotateToUpright();
                upperGripper = gripper1;
                lowerGripper = gripper2;
            }

        } else {
            if (rotate != null) {
                rotateToInverted();
                upperGripper= gripper2;
                lowerGripper = gripper1;
            }
        }
    }

    public void rotateToInverted() {  //check that this direction is right
        rotate.setPosition(0.0);
    }

    public void rotateToUpright() {  //check that this direction is right
        rotate.setPosition(1.0);
    }

    public void stopRotating() {
        rotate.setPosition(0.5);
    }

    public void lowerOpen() {
        Log.d(LOG_TAG, "Achievement Unlocked : Lower Opened");
        lowerGripper.setPosition(GRIPPER_OPEN);
    }

    public void lowerClose() {
        Log.d(LOG_TAG, "Achievement Unlocked : Lower Closed");
        lowerGripper.setPosition(GRIPPER_CLOSED);
    }

    public void upperOpen() {
        Log.d(LOG_TAG, "Achievement Unlocked : Upper Opened");
        upperGripper.setPosition(GRIPPER_OPEN);
    }

    public void upperClose() {
        Log.d(LOG_TAG, "Achievement Unlocked : Upper Closed");
        upperGripper.setPosition(GRIPPER_CLOSED);
    }

    public void maintainUpperGripperPosition() {
       upperGripper.setPosition(upperGripper.getPosition());
    }

    public void maintainLowerGripperPosition() {
       lowerGripper.setPosition(lowerGripper.getPosition());
    }

    public void toggleUpper() {
        // position is floating point, shouldn't compare directly, must use delta
        if (Math.abs(upperGripper.getPosition() - GRIPPER_CLOSED) < 0.05) {
            upperOpen();
        } else {
            upperClose();
        }
    }

    public void toggleLower() {
        // position is floating point, shouldn't compare directly, must use delta
        if (Math.abs(lowerGripper.getPosition() - GRIPPER_CLOSED) < 0.05) {
            lowerOpen();
        } else {
            lowerClose();
        }
    }

    public boolean isUprightLimitReached() {
        if (uprightLimitSwitch == null) {
            return false;
        }

        return !uprightLimitSwitch.getState();
    }

    public boolean isInvertedLimitReached() {
        if (invertedLimitSwitch == null) {
            return false;
        }

        return !invertedLimitSwitch.getState();
    }

    /**
     * enforceLimits() is not working as intended.  It stops rotation when EITHER limit is reached.
     * But we don't want that; when the mechanism starts rotating, it may already be touching the
     * limit switch that it is rotating AWAY from.  Thus, the mechanism never completes its
     * rotation.
     *
     * Adjusted flip() function above to be more like the lift mechanism; it will keep rotating
     * in a given direction until the user input changes.  This required some changes in the
     * RoverRuckusHardware file as well. -- CMN
      */
    public void enforceLimits() {
        if (isUprightLimitReached() || isInvertedLimitReached()) {
            rotate.setPosition(0.5D);
            Log.d(LOG_TAG, "rotation limits reached, stopping servo");
        }
    }

    class Lift {
        private DcMotor liftMotor;
        private DigitalChannel maxHeight;
        private DigitalChannel minHeight;

        public void stop(){
            liftMotor.setPower(0);
        }

        public boolean moveUp(double power){

            if (!maxHeight.getState()) {
                liftMotor.setPower(0);
                return false;
            }
            else{
                liftMotor.setPower(power);

                return true;
            }

        }

        public boolean moveDown(double power) {
            if (!minHeight.getState()) {
                liftMotor.setPower(0);
                return false;
            } else {
                liftMotor.setPower(-power);

                return true;
            }
        }
    }

    public GripperCloseState getGripperCloseState(Telemetry telemetry) {
        return new GripperCloseState(telemetry);
    }

    public GripperOpenState getGripperOpenState(Telemetry telemetry) {
        return new GripperOpenState(telemetry);
    }

    public LiftMoveUpState getLiftMoveUpState(Telemetry telemetry) {
        return new LiftMoveUpState(telemetry);
    }

    class GripperCloseState extends State {

        GripperCloseState(Telemetry telemetry) {
            super("Gripper close", telemetry);
        }

        @Override
        public void resetToStart() {
            // what would it look like if we needed to go back to the state before this one?
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            lowerClose();
            upperClose();

            return nextState;
        }
    }

    class GripperOpenState extends State {

        GripperOpenState(Telemetry telemetry) {
            super("Gripper open", telemetry);
        }

        @Override
        public void resetToStart() {
            // what would it look like if we needed to go back to the state before this one?
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            lowerOpen();
            upperOpen();

            return nextState; // what should we really return?
        }
    }

    class LiftMoveUpState extends TimeoutSafetyState {

        boolean started = false;

        LiftMoveUpState(Telemetry telemetry) {
            super("Lift move up", telemetry, 500 /* millis */);
        }

        @Override
        public void resetToStart() {
            // what would it look like if we needed to go back to the state before this one?
            started = false;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            if (!started) {
                lift.moveUp(.5);
                started = true;
            }

            if (isTimedOut()) {
                lift.stop();

                return nextState;
            }

            return this;
        }
    }
}

