/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.NonNull;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ScoringMechanism {

    public static final long LAUNCHER_TO_SPEED_TIMEOUT_MILLIS = TimeUnit.SECONDS.toMillis(10);

    public static final long WAIT_FOR_HOPPER_TO_FLOAT_MILLIS = 1000; // One second, placeholder

    private Launcher launcher;

    private Intake intake;

    private RevBlinkinLedDriver blinkinLed;

    @Setter
    private RangeInput intakeVelocity;

    @Setter
    private OnOffButton launchTrigger;

    @Setter
    private ToggledButton upToSpeedToggle;

    @Setter
    private OnOffButton unsafe;

    @Setter
    private OnOffButton invertHopper;

    @Setter
    private DebouncedButton stopLauncher;

    private State currentState;

    @Builder
    private ScoringMechanism(HardwareMap hardwareMap,
                            RangeInput intakeVelocity,
                            OnOffButton launchTrigger,
                            Telemetry telemetry,
                            Ticker ticker) {
        launcher = new Launcher(hardwareMap, telemetry, ticker);
        intake = new Intake(hardwareMap, ticker);

        try {
            blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        } catch (IllegalArgumentException iae) {
            Log.e(LOG_TAG, "Can't find blinkin led, either not faked, or not working");
        }

        this.intakeVelocity = intakeVelocity;
        this.launchTrigger = launchTrigger;

        IdleState idleState = new IdleState(telemetry);
        IntakeMoving intakeMoving = new IntakeMoving(telemetry);
        idleState.setIntakeMoving(intakeMoving);
        intakeMoving.setIdleState(idleState);

        IntakeStalled intakeStalled = new IntakeStalled(telemetry, ticker);
        intakeStalled.setIdleState(idleState);
        intakeMoving.setIntakeStalled(intakeStalled);

        PreloadRings preloadRings = new PreloadRings(telemetry, ticker);
        idleState.setPreloadRings(preloadRings);
        intakeMoving.setPreloadRings(preloadRings);
        preloadRings.setIdleState(idleState);

        LauncherToSpeed launcherToSpeed = new LauncherToSpeed(telemetry, ticker);
        preloadRings.setLauncherToSpeed(launcherToSpeed);

        LauncherReady launcherReady = new LauncherReady(telemetry);
        launcherReady.setIdleState(idleState);

        launcherToSpeed.setLauncherReady(launcherReady);
        launcherToSpeed.setIdleState(idleState);

        // because of circular dependencies during construction, we need to post-check
        // that all of the transitions have been setup correctly

        for (ReadyCheckable checkMe : readyCheckables) {
            checkMe.checkReady();
        }

        currentState = idleState;
    }

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

    @VisibleForTesting
    State getCurrentState() {
        return currentState;
    }

    void commonLauncherSpeedHandling() {
        if (upToSpeedToggle.isToggledTrue()) {
            launcher.launcherToFullSpeed();
        } else {
            launcher.launcherToIdleSpeed();
        }
    }

    void unsafeIntakeOperations() {
        // Mostly unsafe because we don't know how to do stall detection from here, yet

        if (unsafe.isPressed()) {
            if (intakeVelocity.getPosition() > 0) {
                intake.intake(intakeVelocity.getPosition());
            } else if (intakeVelocity.getPosition() < 0) {
                intake.outtake(intakeVelocity.getPosition());
            } else {
                intake.stop();
            }
        }
    }

    List<ReadyCheckable> readyCheckables = Lists.newArrayList();

    interface ReadyCheckable {
        void checkReady();
    }

    class IdleState extends NotDebuggableState {
        // Intake: Not Moving 
        // Launcher: Not Moving

        // Transitions - operator requests intake move, operator requests launch of rings
        @Setter
        IntakeMoving intakeMoving;

        @Setter
        PreloadRings preloadRings;

        protected IdleState(Telemetry telemetry) {
            super("Scoring Idle", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            commonLauncherSpeedHandling();

            intake.stop();

            invertHopperFromPulledDown();

            launcher.parkRingFeeder();

            if (intakeVelocity.getPosition() != 0) {
                return intakeMoving;
            } else if (launchTrigger.isPressed()) {
                return preloadRings;
            }

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(intakeMoving);
            Preconditions.checkNotNull(preloadRings);
        }
    }

    private void invertHopperFromPulledDown() {
        if (!invertHopper.isPressed()) {
            launcher.pulldownHopper();
        } else {
            launcher.floatHopperWithLauncher();
        }
    }

    class IntakeMoving extends NotDebuggableState {

        // Intake:Moving
        //  Intake Direction: Operator Controlled 
        // Launcher: Not Moving

        @Setter
        private IdleState idleState;

        @Setter
        private IntakeStalled intakeStalled;

        @Setter
        private PreloadRings preloadRings;

        public IntakeMoving(Telemetry telemetry) {
            super("Intake Moving", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            // Intake encoder not installed yet...

            if (false) {
                if (intake.isStalled()) {
                    intake.resetStallDetector();

                    return intakeStalled;
                }
            }

            commonLauncherSpeedHandling();

            invertHopperFromPulledDown();

            if (intakeVelocity.getPosition() > 0) {
                intake.intake(intakeVelocity.getPosition());

                return this;
            } else if (intakeVelocity.getPosition() < 0) {
                intake.outtake(intakeVelocity.getPosition());

                return this;
            } else if (launchTrigger.isPressed()) {
                return preloadRings;
            }

            return idleState;
        }

        public boolean isIntaking() {
            return intake.isIntaking();
        }

        public boolean isOuttaking() {
            return intake.isOuttaking();
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
            Preconditions.checkNotNull(intakeStalled);
            Preconditions.checkNotNull(preloadRings);
        }
    }

    class IntakeStalled extends StopwatchTimeoutSafetyState implements ReadyCheckable {

        @Setter
        private IdleState idleState;

        public IntakeStalled(Telemetry telemetry, Ticker ticker) {

            super("Intake Stalled", telemetry, ticker, TimeUnit.SECONDS.toMillis(1));
        }

        @Override
        public State doStuffAndGetNextState() {
            intake.stop();

            if (isTimedOut()) {
                resetTimer();

                return idleState;
            }

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad ninjaGamePad) {

        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }
    }

    class PreloadRings extends NotDebuggableState {
        // Intake:Not Moving
        //  Launcher: "Whatever"
        // Hopper: Floats with launcher

        private Stopwatch waitForFloat;

        @Setter
        private LauncherToSpeed launcherToSpeed;

        @Setter
        private IdleState idleState;

        public PreloadRings(Telemetry telemetry, Ticker ticker) {
            super("Preload Rings", telemetry);
            waitForFloat = Stopwatch.createUnstarted(ticker);
        }

        @Override
        public State doStuffAndGetNextState() {
            // Transitions - enough time has passed for hopper to float -> launcher to speed
            //               operator stops requesting launch of rings -> idle state

            intake.stop();
            launcher.launcherToFullSpeed();

            invertHopperFromFloating();

            if (!waitForFloat.isRunning()) {
                waitForFloat.start();
            } else {
                long elapsedWaitMillis = waitForFloat.elapsed(TimeUnit.MILLISECONDS);

                if (elapsedWaitMillis > WAIT_FOR_HOPPER_TO_FLOAT_MILLIS) {
                    waitForFloat.reset(); // for next time!

                    return launcherToSpeed;
                }
            }

            if (stopLauncher.getRise()){
                waitForFloat.reset();

                return idleState;
            }

            unsafeIntakeOperations();

            return this;
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
            Preconditions.checkNotNull(launcherToSpeed);
        }
    }

    class LauncherToSpeed extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        // Intake:Not Moving 
        // Launcher: Moving - accelerate to target velocity, PID (velocity)
        // Hopper - floating

        @Setter
        private LauncherReady launcherReady;

        @Setter
        private IdleState idleState;

        public LauncherToSpeed(Telemetry telemetry, Ticker ticker) {
            super("Launcher to Speed",telemetry, ticker, LAUNCHER_TO_SPEED_TIMEOUT_MILLIS);

            readyCheckables.add(this);
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
            Preconditions.checkNotNull(launcherReady);
        }

        @Override
        public State doStuffAndGetNextState() {

            // Transitions - operator stops requesting launch of rings -> idle
            //               launcher is at speed -> launcher ready

            if (isTimedOut()){
                resetTimer();

                Log.d(LOG_TAG, "Timed out waiting for launcher to get to speed");

                return launcherReady;
            }

            invertHopperFromFloating();

            if (unsafe.isPressed()) {
                Log.w(LOG_TAG, "Unsafe pressed, bypassing launcher speed check");

                return launcherReady;
            }

            if (launcher.isLauncherAtFullSpeed()){
                resetTimer();

                return launcherReady;
            }

            unsafeIntakeOperations();

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    private void invertHopperFromFloating() {
        if (!invertHopper.isPressed()) {
            launcher.floatHopperWithLauncher();
        } else {
            launcher.pulldownHopper();
        }
    }

    class LauncherReady extends NotDebuggableState {
        @Setter
        private IdleState idleState;

        protected LauncherReady(Telemetry telemetry) {
            super("Launcher ready", telemetry);
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(idleState);
        }

        @Override
        public State doStuffAndGetNextState() {
            // FIXME: For *now* would like ability to do "whatever" with intake during launching

            // Intake:Not Moving 
            intake.stop();

            launcher.launcherToFullSpeed();

            // Transitions - operator stops requesting launch of rings (red button?) (back to idle)

            if (launchTrigger.isPressed()) {
                launcher.feedRing();
            } else {
                launcher.parkRingFeeder();

                // Only allowed when not pushing rings
                invertHopperFromFloating();
            }

            if (stopLauncher.getRise()) {
                return idleState;
            }

            return this;
        }

    }

    //
    // Just here to remove some boiler plate code that's not really used by our
    // implementation
    //

    abstract class NotDebuggableState extends State implements ReadyCheckable {

        protected NotDebuggableState(@NonNull String name, Telemetry telemetry) {
            super(name, telemetry);
            readyCheckables.add(this);
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

}
