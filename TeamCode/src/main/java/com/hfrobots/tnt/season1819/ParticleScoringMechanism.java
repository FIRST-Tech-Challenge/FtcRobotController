/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.state.State;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.math.DoubleMath;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.Builder;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ParticleScoringMechanism {
    private static final double ELEVATOR_POWER_LEVEL = 1;

    private static final double kPup = .004;

    private static final double kPdown = .0001;

    private static final double ANTIGRAVITY_FEED_FORWARD = .14;

    private static int ELEVATOR_UPPER_LIMIT_ENCODER_POS = 1550;

    private static int ELEVATOR_LOWER_LIMIT_ENCODER_POS = -130; // FIXME: 12/10 HACK HACK HACK?

    private static int AUTO_STALL_TIMEOUT_SECONDS = 5; // Value based on Kaylin and Lauren observing
                                                       // length of a video of one cycle of the elevator

    private static double OPEN_LOOP_ELEVATOR_POWER_LEVEL = .3;

    private OnOffButton elevatorCommandUpButton;

    private OnOffButton elevatorCommandDownButton;

    private DebouncedButton elevatorUpperLimitButton;

    private DebouncedButton elevatorLowerLimitButton;

    private DebouncedButton elevatorEmergencyStopButton;

    private DcMotorEx elevatorMotor;

    private DigitalChannel upperElevatorLimit;

    private DigitalChannel lowerElevatorLimit;

    private Telemetry telemetry;

    private State currentState;

    // box tipping mechanism

    private Servo boxTipServo;

    private OnOffButton boxTipButton;

    private OnOffButton limitOverrideButton;

    public static double scoringPosition = 1;

    private double loadingPosition = 0;

    private double holdingPosition = scoringPosition / 2;


    @Builder
    public ParticleScoringMechanism(final OnOffButton elevatorCommandUpButton,

                                    final OnOffButton elevatorCommandDownButton,

                                    final DebouncedButton elevatorUpperLimitButton,
                                    final DebouncedButton elevatorLowerLimitButton,
                                    final DebouncedButton elevatorEmergencyStopButton,
                                    final DcMotorEx elevatorMotor,
                                    final DigitalChannel upperElevatorLimit,
                                    final DigitalChannel lowerElevatorLimit,
                                    final Telemetry telemetry,
                                    final Servo boxTipServo,
                                    final OnOffButton boxTipButton,
                                    final OnOffButton limitOverrideButton) {
        this.elevatorCommandUpButton = elevatorCommandUpButton;

        this.elevatorCommandDownButton = elevatorCommandDownButton;

        this.elevatorUpperLimitButton = elevatorUpperLimitButton;

        this.elevatorLowerLimitButton = elevatorLowerLimitButton;

        this.elevatorEmergencyStopButton = elevatorEmergencyStopButton;

        this.elevatorMotor = elevatorMotor;

        this.upperElevatorLimit = upperElevatorLimit;

        this.lowerElevatorLimit = lowerElevatorLimit;

        this.telemetry = telemetry;

        this.boxTipServo = boxTipServo;

        this.boxTipButton = boxTipButton;

        this.limitOverrideButton = limitOverrideButton;

        ElevatorGoUpperLimitState goUpperLimitState = new ElevatorGoUpperLimitState(telemetry);

        ElevatorGoLowerLimitState goLowerLimitState = new ElevatorGoLowerLimitState(telemetry);

        ElevatorDownCommandState downCommandState = new ElevatorDownCommandState(telemetry);

        ElevatorUpCommandState upCommandState = new ElevatorUpCommandState(telemetry);

        ElevatorAtUpperLimitState atUpperLimitState = new ElevatorAtUpperLimitState(telemetry);

        ElevatorAtLowerLimitState atLowerLimitState = new ElevatorAtLowerLimitState(telemetry);

        ElevatorIdleState idleState = new ElevatorIdleState(telemetry);

        goUpperLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
         upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        goLowerLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        downCommandState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        upCommandState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atUpperLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        atLowerLimitState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        idleState.setAllTransitionToStates(goUpperLimitState, goLowerLimitState, downCommandState,
                upCommandState, atLowerLimitState, atUpperLimitState, idleState);

        currentState = atLowerLimitState;

        boxTipServo.setPosition(loadingPosition);
    }

    protected void stopElevator() {
        elevatorMotor.setPower(0);
    }

    protected void elevatorUp() {
        elevatorMotor.setPower(OPEN_LOOP_ELEVATOR_POWER_LEVEL);
    }

    protected void elevatorDown() {
        elevatorMotor.setPower(-OPEN_LOOP_ELEVATOR_POWER_LEVEL);
    }

    public void doPeriodicTask() {
        String tippedState = handleTipAndGate();
        String currentStateName = currentState.getName();

        if (currentStateName != null) {
            currentStateName = currentStateName.replace("Elev-", "");
        } else {
            currentStateName = "Unk";
        }

        telemetry.addData("SM: ", "e: " + currentStateName + " b: " + tippedState +
                (limitOverrideButton.isPressed() ? "!" : ""));

        State nextState = currentState.doStuffAndGetNextState();

        if (nextState != currentState) {
            // We've changed states alert the driving team, log for post-match analysis
            telemetry.addData("00-State", "From %s to %s", currentState, nextState);
            Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                    + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
        }

        currentState = nextState;
    }

    // FIXME: This works for now, but it's not object-oriented. All of these state-specific behaviors
    // belong in the states themselves, and we can take care of *only* the limit override case here!
    // (but it works, and is good enough for league meet #2).
    public String handleTipAndGateOld() {
        // FIXME: What we want to try - is to have the box be "safe" when traveling up or down
        //        or idle *but* be horizontal when in the loading position, or near the loading position
        //        (probably encoder-based measurement to determine that)

        if (boxTipButton.isPressed()) {
            if (limitOverrideButton.isPressed() || isAtUpperLimit()) {
                boxTipServo.setPosition(scoringPosition);

                return "\\";
            }
        } else if (currentStateIsGoingUpOrAtUpperLimit()) {
            if (boxTipButton.isPressed()) {
                boxTipServo.setPosition(scoringPosition);

                return "\\";
            } else {
                boxTipServo.setPosition(holdingPosition);

                return "/";
            }
        } else {
           boxTipServo.setPosition(loadingPosition);

           return "-";
        }

        return "-";
    }

    // FIXME: This works for now, but it's not object-oriented. All of these state-specific behaviors
    // belong in the states themselves, and we can take care of *only* the limit override case here!
    // (but it works, and is good enough for league meet #2).
    public String handleTipAndGate() {
        // First handle box tip button and override - no exceptions

        if (boxTipButton.isPressed()) {
            if (limitOverrideButton.isPressed() || isAtUpperLimit()) {
                boxTipServo.setPosition(scoringPosition);

                return "\\";
            }
        } else if (currentStateIsAtUpperLimit()) {
            boxTipServo.setPosition(holdingPosition);

            return "/";

        } else if (currentStateIsClosedLoopUp()) {
            boxTipServo.setPosition(holdingPosition);

            return "/";

        } else if (currentStateIsClosedLoopDown()) {
            boxTipServo.setPosition(holdingPosition);

            return "/";

        } else if (currentStateIsOpenLoopUpOrDown()) {
            boxTipServo.setPosition(holdingPosition);

            return "/";

        } else if (currentStateIsAtLowerLimit()) {
            int elevatorMotorPosition = elevatorMotor.getCurrentPosition();

            if (elevatorMotorPosition <= 90) {
                boxTipServo.setPosition(loadingPosition);

                return "-";
             } else {
                boxTipServo.setPosition(holdingPosition);

                return "/";
            }

        } else if (currentStateIsIdle()) {
            int elevatorMotorPosition = elevatorMotor.getCurrentPosition();

            if (elevatorMotorPosition <= 90) {
                boxTipServo.setPosition(loadingPosition);

                return "-";

            } else {
                boxTipServo.setPosition(holdingPosition);

                return "/";
            }
        }


        boxTipServo.setPosition(holdingPosition);
        return "/";
    }

    @VisibleForTesting
    public boolean currentStateIsOpenLoopUpOrDown() {
        if (currentState == null) {
            return false;
        }

        boolean currentStateIsUpCommand = currentState.getClass().getName().equals(ElevatorUpCommandState.class.getName());
        boolean currentStateIsDownCommand = currentState.getClass().getName().equals(ElevatorDownCommandState.class.getName());

        return currentStateIsUpCommand || currentStateIsDownCommand;
    }

    @VisibleForTesting
    public boolean currentStateIsIdle() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorIdleState.class.getName());
    }

    @VisibleForTesting
    public boolean currentStateIsAtUpperLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean currentStateIsAtLowerLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorAtLowerLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean currentStateIsClosedLoopUp() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorGoUpperLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean currentStateIsClosedLoopDown() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorGoLowerLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean currentStateIsGoingUpOrAtUpperLimit() {
        if (currentState == null) {
            return false;
        }

        return currentState.getClass().getName().equals(ElevatorGoUpperLimitState.class.getName()) ||
                currentState.getClass().getName().equals(ElevatorAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean isAtUpperLimit() {
        return currentState.getClass().getName().equals(ElevatorAtUpperLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean isAtLowerLimit() {
       return currentState.getClass().getName().equals(ElevatorAtLowerLimitState.class.getName());
    }

    @VisibleForTesting
    public boolean isBoxtipServoInScoringPosition() {
        return DoubleMath.fuzzyEquals(boxTipServo.getPosition(), scoringPosition, 0.01D);
    }

    @VisibleForTesting
    public boolean isBoxtipServoInHoldingPosition() {
        return DoubleMath.fuzzyEquals(boxTipServo.getPosition(), holdingPosition, 0.01D);
    }

    @VisibleForTesting
    public boolean isBoxtipServoInLoadingPosition() {
        return DoubleMath.fuzzyEquals(boxTipServo.getPosition(), loadingPosition, 0.01D);
    }

    abstract class ElevatorBaseState extends TimeoutSafetyState {
        ElevatorGoUpperLimitState goUpperLimitState;

        ElevatorGoLowerLimitState goLowerLimitState;

        ElevatorDownCommandState downCommandState;

        ElevatorUpCommandState upCommandState;

        ElevatorAtUpperLimitState atUpperLimitState;

        ElevatorAtLowerLimitState atLowerLimitState;

        ElevatorIdleState idleState;

        ElevatorBaseState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        protected void setAllTransitionToStates(final ElevatorGoUpperLimitState goUpperLimitState,
                final ElevatorGoLowerLimitState goLowerLimitState,
                final ElevatorDownCommandState downCommandState,
                final ElevatorUpCommandState upCommandState,
                final ElevatorAtLowerLimitState atLowerLimitState,
                final ElevatorAtUpperLimitState atUpperLimitState,
                final ElevatorIdleState idleState) {
            this.goLowerLimitState = goLowerLimitState;

            this.goUpperLimitState = goUpperLimitState; // MM

            this.downCommandState = downCommandState;

            this.upCommandState = upCommandState;

            this.atLowerLimitState = atLowerLimitState;

            this.atUpperLimitState = atUpperLimitState;

            this.idleState = idleState;

            // FIXME: Assert that everything that is required, has been set! (there was a bug hiding in this code!)

        }

        protected State handleButtons() {
            if (elevatorCommandUpButton.isPressed()) {
                return upCommandState;
            } else if (elevatorCommandDownButton.isPressed()) {
                return downCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto upper limit");

                return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()) {
                Log.d(LOG_TAG, "Sensed button press for auto lower limit");

                return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            return null;
        }
    }

    class ElevatorIdleState extends ElevatorBaseState {

        ElevatorIdleState(Telemetry telemetry) {
            super("Elev-idle", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null) {
                return fromButtonState;
            }

            return this; // FIXME: THis isn't correct - what should be returned if nothing has changed?
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    abstract class ElevatorClosedLoopState extends ElevatorBaseState {
        PidController pidController;

        boolean initialized = false;

        ElevatorClosedLoopState(String name, Telemetry telemetry, long timeoutMillis) {
            super(name, telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        protected void setupPidController(double kP) {
            pidController = PidController.builder().setInstanceName("Scoring mechanism pid-controller")
                    .setKp(kP).setAllowOscillation(false)
                    .setTolerance(140)
                    .build();
            pidController.setAbsoluteSetPoint(true);
            pidController.setOutputRange(-ELEVATOR_POWER_LEVEL, ELEVATOR_POWER_LEVEL);
        }
    }

    class ElevatorGoUpperLimitState extends ElevatorClosedLoopState {

        ElevatorGoUpperLimitState(Telemetry telemetry) {
            super("Elev-auto-top", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS));
        }

        @Override
        public State doStuffAndGetNextState() {
            Log.d(LOG_TAG, "Running closed loop state: " + getName());

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return fromButtonState;
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(kPup);

                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(ELEVATOR_UPPER_LIMIT_ENCODER_POS,
                        elevatorMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                stopElevator();

                Log.d(LOG_TAG, "Elevator reached upper target");

                return atUpperLimitState ;
            }

            if (upperElevatorLimit != null && upperElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
                stopElevator();

                return atUpperLimitState;
            }

            Log.d(LOG_TAG, "Elevator setting power via PID to: " + pidOutput);

            double ANTIGRAVITY_OVERCOME_FEED_FORWARD = .33;

            if (pidOutput < ANTIGRAVITY_OVERCOME_FEED_FORWARD) {
                pidOutput = ANTIGRAVITY_OVERCOME_FEED_FORWARD;
            }

            elevatorMotor.setPower(pidOutput);

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }

    }


    class ElevatorGoLowerLimitState extends ElevatorClosedLoopState {

        ElevatorGoLowerLimitState(Telemetry telemetry) {
            super("Elev-auto-bot", telemetry, TimeUnit.SECONDS.toMillis(AUTO_STALL_TIMEOUT_SECONDS)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {

            Log.d(LOG_TAG, "Running closed loop state: " + getName());

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return fromButtonState;
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(kPdown);

                pidController.setOutputRange(-.1, .1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM
                pidController.setTarget(ELEVATOR_LOWER_LIMIT_ENCODER_POS,
                        elevatorMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(elevatorMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Elevator reached lower target");

                stopElevator();

                return atLowerLimitState ;
            }

            if (lowerElevatorLimit != null && lowerElevatorLimit.getState() == false /* digital channels are inverted! */ ) {
                stopElevator();

                return atLowerLimitState;
            }


            elevatorMotor.setPower(pidOutput);

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    class ElevatorAtUpperLimitState extends ElevatorBaseState {

        ElevatorAtUpperLimitState(Telemetry telemetry) {
            super("Elev-@-top", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null && fromButtonState != this &&
                    !fromButtonState.equals(goUpperLimitState) &&
                    !fromButtonState.equals(upCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                elevatorMotor.setPower(0);

                return fromButtonState;
            } else if (fromButtonState != null && limitOverrideButton.isPressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    elevatorMotor.setPower(0);

                    return fromButtonState;
                }
            }

            // FIXME: Let's start with feed-forward only, and see if we really need a PID
            // to hold position

            elevatorMotor.setPower(ANTIGRAVITY_FEED_FORWARD);
            Log.d(LOG_TAG, getName() + " nothing to do, remaining in same state");

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }


    class ElevatorAtLowerLimitState extends ElevatorBaseState {

        ElevatorAtLowerLimitState(Telemetry telemetry) {
            super("Elev-@-bot", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            State fromButtonState = handleButtons();

            if (fromButtonState != null &&  fromButtonState != this &&
                    !fromButtonState.equals(goLowerLimitState) &&
                    !fromButtonState.equals(downCommandState)) {
                Log.d(LOG_TAG, getName() + " responding to buttons and transitioning to " + fromButtonState.getName());

                return fromButtonState;
            } else if (fromButtonState != null && limitOverrideButton.isPressed()) {
                if (fromButtonState != this) {
                    Log.d(LOG_TAG, getName() + " - limits overridden - transitioning to " + fromButtonState.getName());

                    return fromButtonState;
                }
            }

            Log.d(LOG_TAG, getName() + " nothing to do, remaining in same state");

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    class ElevatorDownCommandState extends ElevatorBaseState{

        ElevatorDownCommandState(Telemetry telemetry) {
            super("Elev-cmd-dn", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (elevatorCommandUpButton.isPressed()) {
                    return upCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                    return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()){
                    return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            if (lowerElevatorLimit != null && lowerElevatorLimit.getState() == false) {
                stopElevator();

                return atLowerLimitState;
            } else if (!elevatorCommandDownButton.isPressed()) {
                Log.d(LOG_TAG, "Elevator - down command button released");
                stopElevator();

                return idleState;
            }

            elevatorDown();

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    class ElevatorUpCommandState extends ElevatorBaseState {

        ElevatorUpCommandState(Telemetry telemetry) {
            super("Elev-cmd-up", telemetry, TimeUnit.SECONDS.toMillis(60)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            // State fromButtonState = handleButtons();

            // if (fromButtonState != null && fromButtonState != this) {
            //    Log.d(LOG_TAG, "Transitioning to " + fromButtonState.getName() + " due to button press");
            //    return fromButtonState;
            //}

            if (elevatorCommandDownButton.isPressed()) {
                return downCommandState;
            } else if (elevatorUpperLimitButton.getRise()) {
                return goUpperLimitState;
            } else if (elevatorLowerLimitButton.getRise()){
                return goLowerLimitState;
            } else if (elevatorEmergencyStopButton.getRise()) {
                stopElevator();

                return idleState;
            }

            if (upperElevatorLimit != null && upperElevatorLimit.getState() == false) {
                stopElevator();

                return atUpperLimitState;
            }

            if (!elevatorCommandUpButton.isPressed() /* FiXME: !elevatorCommandDownButton.isPressed()*/) {
                Log.d(LOG_TAG, "Elevator - up command button released");
                stopElevator();

                return idleState;
            }

            elevatorUp();

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }
}
