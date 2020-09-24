/*
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
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
*/

package com.hfrobots.tnt.season1920;

import android.util.Log;

import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.drive.ServoUtil;
import com.hfrobots.tnt.corelib.drive.StallDetector;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StopwatchDelayState;
import com.hfrobots.tnt.corelib.state.StopwatchTimeoutSafetyState;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class DeliveryMechanism {
    Servo shoulderServo;
    Servo wristServo;
    Servo fingerServo;

    ExtendedDcMotor liftMotor;

    ExtendedDcMotor leftIntakeMotor;
    ExtendedDcMotor rightIntakeMotor;

    private DigitalChannel deliveryMechLowLimitSwitch;

    @Setter
    protected RangeInput liftThrottle;

    @Setter
    protected RangeInput intakeThrottle;

    @Setter
    protected DebouncedButton armInPostion;

    @Setter
    protected DebouncedButton armOutPostion;

    @Setter
    protected DebouncedButton rotateWrist;

    @Setter
    protected DebouncedButton grip;

    @Setter
    protected DebouncedButton ungrip;

    @Setter
    protected DebouncedButton stow;

    @Setter
    protected OnOffButton unsafe;

    @Setter
    protected Ticker ticker;

    protected final Telemetry telemetry;

    public final static double SHOULDER_OUT = 0.73;
    public final static double SHOULDER_STOW = 0;
    boolean shoulderFar = false;

    public final static double WRIST_TURN_CLOSE_NATURAL = 1.0;
    public final static double WRIST_TURN_CLOSE_ROTATED = 0.666;
    public final static double WRIST_TURN_FAR_NATURAL = 1.0;
    public final static double WRIST_TURN_FAR_ROTATED = .666;
    public final static double WRIST_STOW = 1.0;
    boolean wristRotated = false;

    public static final double FINGER_INIT = 0.67;
    public final static double FINGER_GRIP = 0.57; // 0.0;
    public final static double FINGER_UNGRIP = 1.0;

    public final static double EJECTION_EJECT_POSITION = 0.328;
    public final static double EJECTION_STOWED_POSITION = 0.0;

    private Servo ejectorServo;

    public DeliveryMechanism(SimplerHardwareMap hardwareMap, Telemetry telemetry, Ticker ticker) {
        this.telemetry = telemetry;
        this.ticker = ticker;

        shoulderServo = hardwareMap.get(Servo.class, "shoulderServo");
        ServoUtil.setupPwmForRevSmartServo(shoulderServo);

        wristServo = hardwareMap.get(Servo.class, "wristServo");
        fingerServo = hardwareMap.get(Servo.class, "fingerServo");
        ejectorServo = hardwareMap.get(Servo.class, "ejectorServo");
        ejectorServo.setPosition(EJECTION_STOWED_POSITION);

        liftMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "liftMotor"));

        leftIntakeMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "leftIntakeMotor"));
        rightIntakeMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "rightIntakeMotor"));


        try {
            deliveryMechLowLimitSwitch = hardwareMap.get(DigitalChannel.class, "deliveryMechLowLimitSwitch");
        } catch (Exception ex) {
            deliveryMechLowLimitSwitch = null;
        }

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Move everything to initial physical state

        if (true) {
            autoStow();
        }

        setupStateMachine();
    }

    public void setIntakeVelocity(double velocity) {
        if (velocity < 0) {
            if (!unsafe.isPressed()) {
                velocity = -0.2;
            }
        }

        leftIntakeMotor.setPower(velocity);
        rightIntakeMotor.setPower(velocity);

        if (velocity < 0) {
            // Safety - there's only certain states
            // where it is safe to eject, namely LoadingState,
            // or when position is above some certain height, let's say 1/2 of the way
            // up?

            if (getCurrentStateName().equals(LoadingState.class.getSimpleName()) ||
                    liftMotor.getCurrentPosition() > LIFT_MAX_HEIGHT_POS / 2) {
                ejectorServo.setPosition(EJECTION_EJECT_POSITION);
            } else {
                telemetry.addData("DM", "Eject servo - not safe");
                ejectorServo.setPosition(EJECTION_STOWED_POSITION);
            }
        } else {
            ejectorServo.setPosition(EJECTION_STOWED_POSITION);
        }
    }

    public void gripBlock() {
        fingerServo.setPosition(FINGER_GRIP);
    }

    public void ungripblock() {
        fingerServo.setPosition(FINGER_UNGRIP);
    }


    private void stowed() {
        shoulderFar = true;
    }

    private void autoStow() {
        State currentState = createStowWristStates(null, telemetry);

        while (currentState != null) {
            currentState = currentState.doStuffAndGetNextState();
            telemetry.update();
        }

        fingerServo.setPosition(FINGER_INIT); // stay within 18"
        telemetry.clear();
    }

    private boolean shouldUseLowerLimit() {
        return true;
    }

    private boolean isLowerLimitReached() {

        boolean lowerLimitReached = !deliveryMechLowLimitSwitch.getState();
        Log.d(LOG_TAG, "Delivery mech lower limit reached: " + lowerLimitReached);
        return lowerLimitReached;
    }

    // FIXME arm out method goes here

    public void turnFar() {
        if (shoulderFar = false){
            wristServo.setPosition(WRIST_TURN_FAR_NATURAL);
            sleepNoThrow(1000);
            shoulderServo.setPosition(SHOULDER_OUT);
            shoulderFar = true;
        } else {
            if (wristRotated)
            {
                wristServo.setPosition(WRIST_TURN_FAR_ROTATED);
            }else{
                wristServo.setPosition(WRIST_TURN_FAR_NATURAL);
            }
        }
    }

    public void rotateToPos() {
         if(shoulderFar == true){
            if (wristRotated == true){
                wristServo.setPosition(WRIST_TURN_FAR_ROTATED);
                wristRotated = false;
            } else {
                wristServo.setPosition(WRIST_TURN_FAR_NATURAL);
                wristRotated = true;
            }
        } else {
            // assumed it is stowed and should not turn at the moment
            wristServo.setPosition(WRIST_STOW);
            wristRotated = false;
        }
    }

    private void sleepNoThrow(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException intEx) {
            // do Nothing
        }
    }

    private State currentState;

    public void periodicTask() {
        if (currentState != null) {
            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                // We've changed states alert the driving team, log for post-match analysis
                //telemetry.addData("00-State", "From %s to %s", currentState, nextState);
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
            }

            currentState = nextState;
        } else {
            Log.e(LOG_TAG, "No state machine setup!");
        }
    }

    public String getCurrentStateName() {
        if (currentState != null) {
            return currentState.getClass().getSimpleName();
        }

        return "NO_CURRENT_STATE";
    }

    protected float getAdjustedLiftThrottlePosition() {
        return  (liftThrottle.getPosition() / 1) * -1.0F;
    }

    protected void setupStateMachine() {
        HomingState homingState = new HomingState(telemetry, ticker);

        StowReturnState stowReturnState = new StowReturnState(telemetry, ticker);

        stowReturnState.setHomingState(homingState);

        PlaceState placeState = new PlaceState(telemetry, ticker,6000);
        placeState.setStowReturnState(stowReturnState);

        AtMinState atMinState = new AtMinState(telemetry, ticker,6000);
        atMinState.setPlaceState(placeState);
        atMinState.setStowReturnState(stowReturnState);

        AtMaxState atMaxState = new AtMaxState(telemetry, ticker,6000);
        atMaxState.setPlaceState(placeState);
        atMaxState.setStowReturnState(stowReturnState);

        ToPlacingDownState toPlacingDownState = new ToPlacingDownState(telemetry, ticker,6000);
        toPlacingDownState.setAtMinState(atMinState);
        toPlacingDownState.setPlaceState(placeState);
        toPlacingDownState.setStowReturnState(stowReturnState);

        atMaxState.setToPlacingDownState(toPlacingDownState);

        ToPlacingUpState toPlacingUpState = new ToPlacingUpState(telemetry, ticker,6000);
        toPlacingUpState.setToPlacingDownState(toPlacingDownState);
        toPlacingUpState.setAtMaxState(atMaxState);
        toPlacingUpState.setPlaceState(placeState);
        toPlacingUpState.setStowReturnState(stowReturnState);

        atMinState.setToPlacingUpState(toPlacingUpState);

        toPlacingDownState.setToPlacingUpState(toPlacingUpState);

        ArmMovingState armMovingState = new ArmMovingState(telemetry, ticker,6000);
        armMovingState.setToPlacingDownState(toPlacingDownState);
        armMovingState.setToPlacingUpState(toPlacingUpState);
        armMovingState.setPlaceState(placeState);
        armMovingState.setAtMaxState(atMaxState);
        armMovingState.setAtMinState(atMinState);

        placeState.setArmMovingState(armMovingState);
        atMaxState.setArmMovingState(armMovingState);
        atMinState.setArmMovingState(armMovingState);

        MoveClearState moveClearState = new MoveClearState(telemetry, ticker,6000);
        moveClearState.setArmMovingState(armMovingState);
        homingState.setMoveClearState(moveClearState);

        LowGripState lowGripState = new LowGripState(telemetry, ticker,6000);
        lowGripState.setMoveClearState(moveClearState);

        LoadingState loadingState = new LoadingState(telemetry,ticker,6000);
        loadingState.setLowGripState(lowGripState);
        lowGripState.setLoadingState(loadingState);
        stowReturnState.setLoadingState(loadingState);
        homingState.setLoadingState(loadingState);

        // because of circular dependencies during construction, we need to post-check
        // that all of the transitions have been setup correctly

        for (ReadyCheckable checkMe : readyCheckables) {
            checkMe.checkReady();
        }

        currentState = homingState;
    }

    List<ReadyCheckable> readyCheckables = Lists.newArrayList();

    abstract class DeliveryMechanismState extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        public DeliveryMechanismState(String name, Telemetry telemetry, Ticker ticker, long safetyTimeoutMillis) {
            super(name, telemetry, ticker, safetyTimeoutMillis);

            readyCheckables.add(this);
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class LoadingState extends DeliveryMechanismState  {
        @Setter
        private LowGripState lowGripState;

        private boolean initialized = false;

        public LoadingState(Telemetry telemetry,
                            Ticker ticker,
                            long safetyTimeoutMillis) {
            super("Loading State", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (lowGripState == null) {
                throw new IllegalArgumentException("lowGripState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // encoder is in low position
            // arm is in stow position and movement from position isn't allowed
            // fingers are ungripped

            if (!initialized) {
                // arm is in stow position

                // FIXME: Do that ^

                // fingers are ungripped

                ungripblock();

                initialized = true;
            }

            if (grip.getRise()) {
                // Normally-allowed transitions
                gripBlock(); // FIXME: Do you want the grip to happen in the state that is known as "grip"?

                initialized = false;

                return lowGripState;
            } else if (ungrip.getRise()) {
                ungripblock();

                return this;
            } if (unsafe.isPressed()) {
                // Unsafe override transitions
                Log.w(LOG_TAG, "Using unsafe override!");

                initialized = false;

                return this; // FIXME: Probably not what you want
            } else {
                return this;
            }
        }
    }

    class LowGripState extends DeliveryMechanismState {
        @Setter
        private MoveClearState moveClearState;

        @Setter
        private LoadingState loadingState;


        public LowGripState(Telemetry telemetry, Ticker ticker, long safetyTimeoutMillis) {
            super("Low grip state", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (moveClearState == null) {
                throw new IllegalArgumentException("moveClearState not set");
            }

            if (loadingState == null) {
                throw new IllegalArgumentException("loadingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // encoder is in low position

            // arm is in stow position and movement from position isn't allowed

            // FIXME: Do this ^

            // fingers are gripped
            gripBlock();

            float liftThrottlePos = getAdjustedLiftThrottlePosition();

            if (liftThrottlePos > 0) {
                return moveClearState;
            } else if (ungrip.getRise()) {
                ungripblock();

                return loadingState;
            } else {
                return this;
            }
        }
    }

    private static final double LIFT_POWER_LEVEL = 1;

    public final static int LIFT_CLEAR_SUPERSTRUCTURE_POS = 0;

    public final static int LIFT_MAX_HEIGHT_POS = 2750;

    public final static int LIFT_MIN_HEIGHT_POS = 0;

    public final static float LIFT_HOLD_FEED_FORWARD = 0.05F;

    public final static int MAX_ENCODER_DRIFT_FOR_HOME_POS = 15; // approx 1/4" according to Lauren

    private static final double kPdown = .01;

    // An example of how to do this is in last year's code,
    // Cmd + B and look for "ElevatorGoUpperLimitState"
    //
    // We'll need a target encoder value (for now, just guess, but make it
    // a constant) to represent the point where the elevator needs to go to be
    // clear of the robot superstructure
    //
    // Once we have this one implemented, there are many others that follow
    class MoveClearState extends DeliveryMechanismState {
        @Setter
        private ArmMovingState armMovingState;

        public MoveClearState(Telemetry telemetry,
                              Ticker ticker,
                              long safetyTimeoutMillis) {
            super("Move clear state", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (armMovingState == null) {
                throw new IllegalArgumentException("armMovingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState(){
            float liftThrottlePos = getAdjustedLiftThrottlePosition();

            if (ungrip.getRise()) {
                ungripblock();

                return this;
            }

            // MM: Remove me
            if (liftMotor.getCurrentPosition() >= LIFT_CLEAR_SUPERSTRUCTURE_POS /* check encoder - are we clear? */) {
                return armMovingState;
            } else {
                final double motorPower;

                if (liftThrottlePos > 0 || liftThrottlePos < 0) {
                    motorPower = liftThrottlePos;
                } else {
                    // Set motor power to feed-forward value to hold position
                    motorPower = LIFT_HOLD_FEED_FORWARD;
                }

                liftMotor.setPower(motorPower);

                return this;
            }
        }
    }

    class ArmMovingState extends DeliveryMechanismState {
        @Setter
        private ToPlacingDownState toPlacingDownState;

        @Setter
        private ToPlacingUpState toPlacingUpState;

        @Setter
        private PlaceState placeState;

        @Setter
        private AtMinState atMinState;

        @Setter
        private AtMaxState atMaxState;

        public ArmMovingState(Telemetry telemetry,
                              Ticker ticker,
                              long safetyTimeoutMillis) {
            super("Arm Moving state", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (toPlacingUpState == null) {
                throw new IllegalArgumentException("toPlacingUpState not set");
            }

            if (toPlacingDownState == null) {
                throw new IllegalArgumentException("toPlacingDownState not set");
            }

            if (placeState == null) {
                throw new IllegalArgumentException("placeState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            float liftThrottlePos = getAdjustedLiftThrottlePosition();

            // For first league meet, let's simplify, and have this state do up/down
            // by throttle control, not lift to specific levels!

            if (liftThrottlePos < 0 /*down command given*/) {
                if (unsafe.isPressed()) {
                     // FIXME: What would we allow? Down with limit switch check to transition
                    // to loading?
                }

                // Always check limits first!
                // FIXME: Needs a test

                if (liftMotor.getCurrentPosition() <= LIFT_CLEAR_SUPERSTRUCTURE_POS) {
                    liftMotor.setPower(LIFT_HOLD_FEED_FORWARD);

                    return atMinState;
                }

                liftMotor.setPower(liftThrottlePos * .5);

                return this;
            } else if (liftThrottlePos > 0 /*up command given*/) {
                // Always check limits first!
                // FIXME: Needs a test
                if (liftMotor.getCurrentPosition() >= LIFT_MAX_HEIGHT_POS) {
                    liftMotor.setPower(LIFT_HOLD_FEED_FORWARD);

                    return atMaxState;
                }

                liftMotor.setPower(liftThrottlePos);

                return this;
            } else {
                // Need feed-forward on lift motor to hold position if not moving up/down
                liftMotor.setPower(LIFT_HOLD_FEED_FORWARD);

                // For first league meet, we would allow stow return and to placing
                // transitions!?

                return placeState;
            }
        }
    }

    class ToPlacingDownState extends DeliveryMechanismState {
        @Setter
        private AtMinState atMinState;

        @Setter
        private StowReturnState stowReturnState;

        @Setter
        private PlaceState placeState;

        @Setter
        private ToPlacingUpState toPlacingUpState;

        public ToPlacingDownState(Telemetry telemetry,
                                  Ticker ticker,
                                  long safetyTimeoutMillis) {
            super("to placing down state", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (atMinState == null) {
                throw new IllegalArgumentException("atMinState not set");
            }

            if (stowReturnState == null){
                throw new IllegalArgumentException("stowReturnState not set");
            }

            if(placeState == null){
                throw new IllegalArgumentException("placeState not set");
            }

            if (toPlacingUpState == null){
                throw new IllegalArgumentException("toPlacingUpState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // encoder is in low position
            // arm is in stow position and movement from position isn't allowed
            // fingers are gripped

            // FIXME: Need to check encoder on lift motor to see if at or below "clear" point
            // and stop motor, transition to AtMinState??

            if (false /*"place" button is pressed*/) {
                return placeState;
            } else if (false/*"stow" button is pressed*/) {
                return stowReturnState;
            } else if (false/*reach min pos*/) { // see above, check this *FIRST*
                return atMinState;
            } else if (false/*"up" button pressed*/) {
                return toPlacingUpState;
            } else {
                return this;
            }
        }
    }

    class ToPlacingUpState extends DeliveryMechanismState{
        @Setter
        private AtMaxState atMaxState;

        @Setter
        private PlaceState placeState;

        @Setter
        private StowReturnState stowReturnState;

        @Setter
        private ToPlacingDownState toPlacingDownState;

        public ToPlacingUpState(Telemetry telemetry,
                                Ticker ticker,
                                long safetyTimeoutMillis) {
            super("To placing up state", telemetry, ticker, safetyTimeoutMillis);

        }

        public void checkReady() {
            if (atMaxState == null) {
                throw new IllegalArgumentException("atMaxState not set");
            }

            if (placeState == null){
                throw new IllegalArgumentException("placingState not set");
            }

            if (stowReturnState == null){
                throw new IllegalArgumentException("stowReturnState not set");
            }

            if (toPlacingDownState == null){
                throw new IllegalArgumentException("toPlacingDownState");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // encoder is < max position
            // arm is in position from last state and movement from position isn't allowed

            // fingers are gripped
            gripBlock();

            if (false /*reached max*/) {
                // FIXME: ^ Check encoder, if reached, stop lift motor before transition
                return atMaxState;
            } else if (false /*"down" button is presse*/) {
                return toPlacingDownState;
            } else if (false/*placing button pressed*/) {
                return placeState;
            } else if (false /*stow button pressed*/) {
                return stowReturnState;
            } else {
                return this;
            }
        }
    }

    class AtMaxState extends DeliveryMechanismState {
        @Setter
        private  ToPlacingDownState toPlacingDownState;

        @Setter
        private  StowReturnState stowReturnState;

        @Setter
        private  PlaceState placeState;

        @Setter
        private  ArmMovingState armMovingState;

        public AtMaxState(Telemetry telemetry,
                          Ticker ticker,
                          long safetyTimeoutMillis) {
            super("at max state", telemetry, ticker, safetyTimeoutMillis);

        }

        public void checkReady() {
            if (toPlacingDownState == null) {
                throw new IllegalArgumentException("toPlacingDownState not set");
            }
            if (stowReturnState == null) {
                throw new IllegalArgumentException("stowReturnState not set");
            }
            if (placeState == null) {
                throw new IllegalArgumentException("placeState not set");
            }
            if (armMovingState == null) {
                throw new IllegalArgumentException("armMovingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            if (getAdjustedLiftThrottlePosition() < 0) {
                return armMovingState;
            } else if (stow.getRise()) {
                return stowReturnState;
            } else {
                dealWithControlsWhileStationary();

                return this;
            }
        }
    }

    private void dealWithControlsWhileStationary() {
        liftMotor.setPower(LIFT_HOLD_FEED_FORWARD);

        if (armOutPostion.getRise()) {
            shoulderServo.setPosition(SHOULDER_OUT);
            shoulderFar = true;
        } else if (rotateWrist.getRise()) {
            if (shoulderFar) {
                if (wristRotated == true){
                    wristServo.setPosition(WRIST_TURN_FAR_ROTATED);
                    wristRotated = false;
                } else {
                    wristServo.setPosition(WRIST_TURN_FAR_NATURAL);
                    wristRotated = true;
                }
            }
        } else if (ungrip.getRise()) {
            // FIXME: Prevent dropping block if arm is not out
            ungripblock();
        } else if (grip.getRise()) {
            gripBlock();
        }
    }

    class AtMinState extends DeliveryMechanismState{
        @Setter
        private ToPlacingUpState toPlacingUpState;

        @Setter
        private StowReturnState stowReturnState;

        @Setter
        private ArmMovingState armMovingState;

        @Setter
        private PlaceState placeState;

        public AtMinState(Telemetry telemetry,
                          Ticker ticker,
                          long safetyTimeoutMillis
                          ) {
            super("At Min", telemetry, ticker, safetyTimeoutMillis);

        }

        public void checkReady() {
            if (toPlacingUpState == null) {
               throw new IllegalArgumentException("toPlacingUpState not set");
            }
            if (stowReturnState == null) {
                throw new IllegalArgumentException("stowReturnState not set");
            }
            if (placeState == null) {
                throw new IllegalArgumentException("placeState not set");
            }
            if (armMovingState == null) {
                throw new IllegalArgumentException("armMovingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // lift cannot be lowered any further
            // feed-forward applied to hold position

            if (getAdjustedLiftThrottlePosition() > 0) {
                return armMovingState;
            } else if (stow.getRise()) {
                return stowReturnState;
            } else {
                dealWithControlsWhileStationary();

                return this;
            }
        }
    }

    class PlaceState extends DeliveryMechanismState {
        @Setter
        private StowReturnState stowReturnState;

        @Setter
        private ArmMovingState armMovingState;

        public PlaceState(Telemetry telemetry,
                          Ticker ticker,
                          long safetyTimeoutMillis) {
            super("place state", telemetry, ticker, safetyTimeoutMillis);
        }

        public void checkReady() {
            if (stowReturnState == null) {
                throw new IllegalArgumentException("stowReturnState not set");
            }

            if (armMovingState == null) {
                throw new IllegalArgumentException("armMovingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            if (stow.getRise() /* "return" button is pressed */) {
                return stowReturnState;
            } else if (getAdjustedLiftThrottlePosition() != 0) {
                return armMovingState;
            } else {
                dealWithControlsWhileStationary();

                return this;
            }
        }
    }

    class StowReturnState extends DeliveryMechanismState {
        @Setter
        private LoadingState loadingState;

        @Setter
        private HomingState homingState;

        private boolean initialized = false;

        private State stowingArmState = null;

        private PidController pidController;

        boolean pidInitialized = false;

        private StallDetector stallDetector;

        public StowReturnState(Telemetry telemetry, Ticker ticker) {
            super("stow return state", telemetry, ticker, TimeUnit.SECONDS.toMillis(3));

        }

        public void checkReady() {
            if (loadingState == null) {
                throw new IllegalArgumentException("loadingState not set");
            }

            if (homingState == null) {
                throw new IllegalArgumentException("homingState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // return arm to stow state (give time to happen)

            if (!initialized) {
                Log.d(LOG_TAG, "Running states for stowing the arm");

                ungripblock();

                stowingArmState = createStowWristStates(null, telemetry);
                initialized = true;
                stallDetector = new StallDetector(ticker, 1, TimeUnit.SECONDS.toMillis(2));
            }

            if (unsafe.isPressed()) {
                // FIXME: What do we allow here? Throttle up/down goes to arm-moving?
            }

            if (stowingArmState != null) {
                // maintain feed-forward
                liftMotor.setPower(LIFT_HOLD_FEED_FORWARD);

                State nextState = stowingArmState.doStuffAndGetNextState();

                if (nextState != null) {
                    stowingArmState = nextState;

                    return this;
                }

                Log.d(LOG_TAG, "Finished states for stowing the arm");
                stowingArmState = null;
            }

            // FIXME: Start looking for timeout at this point, since the we get stuck waiting
            // for the PID sometimes...

            if (isTimedOut()) {
                Log.d(LOG_TAG, "Error: Timed out while trying to reach bottom");

                liftMotor.setPower(0);

                pidInitialized = false;

                initialized = false;

                return homingState;
            }

            if (shouldUseLowerLimit() && isLowerLimitReached()) {
                Log.d(LOG_TAG, "Error: Limit switch reached before encoder count");

                liftMotor.setPower(0);

                pidInitialized = false;

                initialized = false;

                return homingState;
            }

            // After arm is stowed, lift moves down to loading position - then transitions to loading state

            if (!pidInitialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(kPdown);

                pidController.setOutputRange(-1, .7); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(LIFT_MIN_HEIGHT_POS - 100,
                        liftMotor.getCurrentPosition());

                pidInitialized = true;
            }

            int currentPosition = liftMotor.getCurrentPosition();

            if (stallDetector != null && stallDetector.isStalled(currentPosition)) {
                Log.d(LOG_TAG, "Stall detected! lethal force engaged");

                liftMotor.setPower(0);

                pidInitialized = false;

                initialized = false;

                return homingState;
            }

            double pidOutput = pidController.getOutput(currentPosition);

            boolean pidOnTarget = pidController.isOnTarget();

            if (pidOnTarget) {
                Log.d(LOG_TAG, "Lift PID reached lower target");

                liftMotor.setPower(0);

                pidInitialized = false;

                initialized = false;

                return homingState;
            }

            liftMotor.setPower(pidOutput);

            return this;
        }

        protected void setupPidController(double kP) {
            pidController = PidController.builder().setInstanceName("Delivery mechanism pid-controller")
                    .setKp(kP).setAllowOscillation(false)
                    .setTolerance(140)
                    .build();
            pidController.setAbsoluteSetPoint(true);
            pidController.setOutputRange(-LIFT_POWER_LEVEL, LIFT_POWER_LEVEL);
        }
    }

    class HomingState extends DeliveryMechanismState {
        @Setter
        private LoadingState loadingState;

        @Setter
        private MoveClearState moveClearState;


        public HomingState(Telemetry telemetry, Ticker ticker) {
            super("Homing", telemetry, ticker, TimeUnit.SECONDS.toMillis(10));

        }

        public void checkReady() {
            if (loadingState == null) {
                throw new IllegalArgumentException("loadingState not set");
            }

            if (moveClearState == null) {
                throw new IllegalArgumentException("moveClearState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            // FIXME! FIXME! FIXME: We're accumulating time for every time we enter+exit this
            // state, which eventually causes it to fail!

            if (isTimedOut()) {
                Log.d(LOG_TAG, "Timed out waiting for limit switch");

                liftMotor.setPower(0);

                resetTimer();

                return loadingState;
            }

            if (!shouldUseLowerLimit()) {
                Log.d(LOG_TAG, "Not sure we have a limit switch, assuming lift is homed");

                resetTimer();

                return loadingState;
            }

            if (isLowerLimitReached()) {
                Log.d(LOG_TAG, "Lift lower limit switch sensed, lift is homed");

                liftMotor.setPower(0);

                int currentPosition = liftMotor.getCurrentPosition();

                // TODO: It would be awesome to emit this as a metric over time
                int lowLimitEncoderDrift = Math.abs(currentPosition - LIFT_MIN_HEIGHT_POS);

                if (lowLimitEncoderDrift > MAX_ENCODER_DRIFT_FOR_HOME_POS) {
                    Log.d(LOG_TAG, String.format("Encoder drift of %d is > than limit of %d, resetting encoders",
                            lowLimitEncoderDrift, MAX_ENCODER_DRIFT_FOR_HOME_POS));

                    liftMotor.resetLogicalEncoderCount();
                }

                resetTimer();

                return loadingState;
            }

            if (unsafe.isPressed()) {
                float liftThrottlePos = getAdjustedLiftThrottlePosition();

                if (liftThrottlePos > 0) {
                    resetTimer();

                    return moveClearState;
                } else if (ungrip.getRise()) {
                    ungripblock();

                    return this;
                } else if (grip.getRise()) {
                    gripBlock();

                    return this;
                }
            }

            liftMotor.setPower(-.5);

            return this;
        }
    }

    // FIXME! There's a lot of duplicated code in here that we could clean up...

    State createStowWristStates(final State lastState, Telemetry telemetry) {
        // StopwatchDelayState waitForWristState = new StopwatchDelayState("Wait for wrist", telemetry, ticker, 1, TimeUnit.SECONDS);

        // StowWristState stowWristState = new StowWristState(waitForWristState, telemetry, 60000);

        StopwatchDelayState waitForShoulderState = new StopwatchDelayState("Wait for shoulder", telemetry, ticker,1, TimeUnit.SECONDS);

        StowShoulderState stowShoulderState = new StowShoulderState(waitForShoulderState, telemetry, ticker,60000);

        // waitForWristState.setNextState(stowShoulderState);

        StopwatchDelayState waitForFingerState = new StopwatchDelayState("Wait for finger", telemetry,  ticker, 250, TimeUnit.MILLISECONDS);

        FingerUngripState fingerUngripState = new FingerUngripState(waitForFingerState, telemetry, ticker,60000);

        waitForShoulderState.setNextState(fingerUngripState);

        waitForFingerState.setNextState(lastState);

        return stowShoulderState;
    }

    class StowWristState extends DeliveryMechanismState {
        private StopwatchDelayState waitForWristState;

        public StowWristState(StopwatchDelayState waitForWristState, Telemetry telemetry,
                              Ticker ticker,
                              long safetyTimeoutMillis) {
            super("stow wrist", telemetry, ticker, safetyTimeoutMillis);
            this.waitForWristState = waitForWristState;
        }

        public void checkReady() {
            if (waitForWristState == null) {
                throw new IllegalArgumentException("waitForWristState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            wristServo.setPosition(WRIST_STOW);

            return waitForWristState;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }


    class StowShoulderState extends DeliveryMechanismState {
        private StopwatchDelayState waitForShoulderState;

        public StowShoulderState(StopwatchDelayState waitForShoulderState, Telemetry telemetry,
                                 Ticker ticker, long safetyTimeoutMillis) {
            super("stow shoulder", telemetry, ticker, safetyTimeoutMillis);
            this.waitForShoulderState = waitForShoulderState;
        }

        public void checkReady() {
            if (waitForShoulderState == null) {
                throw new IllegalArgumentException("waitForShoulderState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            shoulderServo.setPosition(SHOULDER_STOW);

            return waitForShoulderState;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class FingerUngripState extends DeliveryMechanismState {

        private StopwatchDelayState waitForFingerState;

        public FingerUngripState(StopwatchDelayState waitForFingerState, Telemetry telemetry,
                                 Ticker ticker, long safetyTimeoutMillis) {
            super("ungrip finger", telemetry, ticker, safetyTimeoutMillis);
            this.waitForFingerState = waitForFingerState;
        }

        public void checkReady() {
            if (waitForFingerState == null) {
                throw new IllegalArgumentException("waitForFingerState not set");
            }
        }

        @Override
        public State doStuffAndGetNextState() {
            ungripblock();

            stowed();

            return waitForFingerState;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }
}
