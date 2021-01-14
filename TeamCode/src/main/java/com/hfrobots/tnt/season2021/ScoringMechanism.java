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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.NonNull;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ScoringMechanism {

    public static final long LAUNCHER_TO_SPEED_TIMEOUT_MILLIS = TimeUnit.SECONDS.toMillis(2);

    private Launcher launcher;
    private Intake intake;

    @Setter
    private RangeInput intakeVelocity;

    @Setter
    private OnOffButton launchTrigger;

    private State currentState;

    @Builder
    private ScoringMechanism(HardwareMap hardwareMap,
                            RangeInput intakeVelocity,
                            OnOffButton launchTrigger,
                            Telemetry telemetry,
                            Ticker ticker) {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);

        this.intakeVelocity = intakeVelocity;
        this.launchTrigger = launchTrigger;

        IdleState idleState = new IdleState(telemetry);
        IntakeMoving intakeMoving = new IntakeMoving(telemetry);
        idleState.setIntakeMoving(intakeMoving);
        intakeMoving.setIdleState(idleState);

        PreloadRings preloadRings = new PreloadRings(telemetry);
        idleState.setPreloadRings(preloadRings);

        LauncherToSpeed launcherToSpeed = new LauncherToSpeed(telemetry, ticker);

        LauncherReady launcherReady = new LauncherReady(telemetry);

        // MM - Remove Me for Kids to Look Into
        launcherToSpeed.setLauncherReady(launcherReady);

        FeedRingIntoLauncher feedRingIntoLauncher = new FeedRingIntoLauncher();

        DelayBeforeFeedingAgain delayBeforeFeedingAgain = new DelayBeforeFeedingAgain();

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
            launcher.stop();
            intake.stop();

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

    class IntakeMoving extends NotDebuggableState {

        // Intake:Moving
        //  Intake Direction: Operator Controlled 
        // Launcher: Not Moving

        // Transitions - operator stops requesting intake to move, operator requests launch of rings
        @Setter
        private IdleState idleState;

        public IntakeMoving(Telemetry telemetry) {
            super("Intake Moving", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (intakeVelocity.getPosition() > 0) {
                intake.intake(intakeVelocity.getPosition());
            } else if (intakeVelocity.getPosition() < 0) {
                intake.outtake(intakeVelocity.getPosition());
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
        }
    }

    class PreloadRings extends NotDebuggableState{
        // Intake:Not Moving
        //  Launcher: Not Moving*

        // Transitions - operator stops requesting launch of rings

        public PreloadRings(Telemetry telemetry) {
            super("Preload Rings", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            return null;
        }

        @Override
        public void checkReady() {
            //Preconditions.checkNotNull(idleState);
        }
    }

    class LauncherToSpeed extends StopwatchTimeoutSafetyState implements ReadyCheckable {
        // Intake:Not Moving 
        // Launcher: Moving - accelerate to target velocity, PID (velocity)

        // Transitions - operator stops requesting launch of rings, launcher is at speed

        private PidController pid;
        private double kP;
        private double kI;
        private double kF;
        private double encoderClicksPerSec;

        private int lastEncoderCount = 0;

        private Stopwatch stopwatch = Stopwatch.createUnstarted();

        @Setter
        private LauncherReady launcherReady;

        protected ExtendedDcMotor motor;

        public LauncherToSpeed(Telemetry telemetry, Ticker ticker) {
            super("Launcher to Speed",telemetry, ticker, LAUNCHER_TO_SPEED_TIMEOUT_MILLIS);

            pid = PidController.builder().setKp(kP).setkI(kI).setkF(kF).setAllowOscillation(true)
                    .setTolerance(encoderClicksPerSec*.03).build();
            pid.setAbsoluteSetPoint(true);
            pid.setTarget(encoderClicksPerSec, 0);

            readyCheckables.add(this);
        }

        @Override
        public void checkReady() {
            Preconditions.checkNotNull(launcherReady);
        }

        @Override
        public State doStuffAndGetNextState() {

            // FIXME: Transition for operator stops asking for ring launch

            if (isTimedOut()){
                resetTimer();
                return launcherReady;
            }

            if(pid.isOnTarget()){
                return launcherReady;
            }

            if (!stopwatch.isRunning()) {
                stopwatch.start();
                lastEncoderCount = motor.getCurrentPosition();

                motor.setPower(pid.getOutput(0));

                return this;
            }

            long elapsedMs = stopwatch.elapsed(TimeUnit.MILLISECONDS);

            if (elapsedMs > 100) {
                stopwatch.reset();
                stopwatch.start();

                int deltaEncoderCount = motor.getCurrentPosition() - lastEncoderCount;


                double currEncoderPerSec = (double)deltaEncoderCount / elapsedMs * TimeUnit.SECONDS.toMillis(1);

                double motorPower = pid.getOutput(currEncoderPerSec);

                motor.setPower(motorPower);
            }

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }

    class LauncherReady extends NotDebuggableState {
        //FIX ME

        protected LauncherReady(Telemetry telemetry) {
            super("Launcher ready", telemetry);
        }

        @Override
        public void checkReady() {

        }

        @Override
        public State doStuffAndGetNextState() {
            return null;
        }
        // Intake:Not Moving 
        // Launcher: Moving - hold voltage

        // Transitions - operator stops requesting launch of rings (back to idle)

    }

    class FeedRingIntoLauncher {
        // Intake:Not Moving 
        // Launcher: Moving - hold voltage 
        // Ring Feeder: Feeding

        // Transitions: Delay after feeding -> delay before feeding again
    }

    class DelayBeforeFeedingAgain {
        // Intake:Not Moving 
        // Launcher: Moving - hold voltage 
        // Ring Feeder: Not Feeding

        // Transitions - time expires (go back to launcher-to-speed),
        //               operator stops requesting launch of rings (back to idle)
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
