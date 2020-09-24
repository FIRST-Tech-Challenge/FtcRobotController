/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1617;


import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.drive.CheesyDrive;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.hfrobots.tnt.corelib.state.ToggleState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

/**
 * Provide a basic manual operational mode that controls the tank drive.
 */
@TeleOp(name="00-VV Teleop")
@Disabled
@SuppressWarnings("unused")
public class VelocityVortexTeleop extends VelocityVortexHardware

{
    protected boolean useEncoders = false;

    private State collectorToggleState;

    private State collectorReverseToggleState;

    private StateMachine particleShooterStateMachine;

    private StateMachine ballGrabberStateMachine;

    private CheesyDrive cheesyDrive;

    /*
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public VelocityVortexTeleop() {
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        super.init();

        // No "runaway robot"
        drive.drivePower(0, 0);
        liftMotor.setPower(0);
        topParticleShooter.setPower(0);
        bottomParticleShooter.setPower(0);


        if (!useEncoders) {
            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        collectorToggleState = new ToggleState("Collector inwards", telemetry, collectorToggleButton) {
            @Override
            protected void toggledOn() {
                Log.d("VV", "Collector toggled inwards(off->on)");

                runParticleCollectorInwards();
            }

            @Override
            protected void toggledOff() {
                Log.d("VV", "Collector toggled inwards(on->off)");

                particleCollectorOff();
            }
        };

        collectorReverseToggleState = new ToggleState("Collector outwards", telemetry, collectorReverseToggleButton) {
            @Override
            protected void toggledOn() {
                Log.d("VV", "Collector toggled outwards(off->on)");

                runParticleCollectorOutwards();
            }

            @Override
            protected void toggledOff() {
                Log.d("VV", "Collector toggled outwards(on->off)");

                particleCollectorOff();
            }
        };

        particleShooterStateMachine = createShooterStateMachineForTeleop();

        cheesyDrive = new CheesyDrive(telemetry, drive,
                driversGamepad.getLeftStickY(), driversGamepad.getRightStickX(),
                driversGamepad.getAButton(), directionFlip, brakeNoBrake, halfSpeed);
        ballGrabberStateMachine = createBallGrabberStateMachine();
    }

    @Override
    public void start() {
        super.start();
        logBatteryState("Teleop.start()");
        beaconPusherUnderColorSensor.setPosition(0);
        beaconPusherNoColorSensor.setPosition(0);
    }

    @Override
    public void stop() {
        super.stop();
        logBatteryState("Teleop.stop()");
    }

    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     * <p>
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop()

    {
        handleDrive();
        handleCollector();
        handleParticleShooter();
        handleBallGrabber();
        handleLift();
        updateGamepadTelemetry();
    }

    private void handleBallGrabber() {
        ballGrabberStateMachine.doOneStateLoop();
    }

    /**
     * Runs the state machine for the particle shooter
     */
    private void handleParticleShooter() {
        // FIXME Figure out how to make these work together
        //if (shooterTrigger.isPressed()) {
        //    shooterOn();
        //} else {
        //    shooterOff();
        //}
        particleShooterStateMachine.doOneStateLoop();
    }

    /**
     * Runs the state machine for the collector mechanism
     */
    private void handleCollector() {
        collectorToggleState = collectorToggleState.doStuffAndGetNextState();
        collectorReverseToggleState = collectorReverseToggleState.doStuffAndGetNextState();
    }

    private boolean liftSafetyPressed = false; // only log once on each state transition

    private boolean autoUnlocked = true;

    private void handleLift() {
        if (liftSafety.isPressed()) {
            if (!liftSafetyPressed) {
                Log.d("VV", "Lift safety trigger pressed");

                liftSafetyPressed = true;
            }


            float liftThrottlePosition = liftThrottle.getPosition();

            // If the lift is going to move, and we haven't auto-unlocked, then
            // do so!
            if (Math.abs(liftThrottlePosition) >= .1 && !autoUnlocked) {
                Log.d("VV", "Auto-unlocking forks");

                unlockForks();
                autoUnlocked = true;
            }

            liftMotor.setPower(liftThrottlePosition);

            if (liftUnlockButton.getRise()) {
                Log.d("VV", "Unlocking forks");

                unlockForks();
            }

            float rightStickYPosition = operatorsGamepad.getRightStickY().getPosition();

            if (rightStickYPosition < 0) {
                //Log.d("VV", "Tilting forks back");

                tiltForksBack(Math.abs(rightStickYPosition) * .025);
            } else if (rightStickYPosition > 0) {
                //Log.d("VV", "Tilting forks forward");

                tiltForksForward(Math.abs(rightStickYPosition) * 0.25);
            }
        } else {
            if (liftSafetyPressed) {
                liftSafetyPressed = false;

                Log.d("VV", "Lift safety trigger released");
            }

            liftMotor.setPower(0);
        }
    }

    protected StateMachine createShooterStateMachineForTeleop() {
        StateMachine shooterStateMachine = new StateMachine(telemetry);

        State waitingForButtonPressState = new WaitForButton(particleShooterBouncy, telemetry);
        State waitingForButtonReleaseState = new WaitForButtonRelease(particleShooterBouncy, telemetry);

        addShooterStateMachine(shooterStateMachine, waitingForButtonPressState,
                waitingForButtonReleaseState, new CollectorOffState(telemetry), true);

        return shooterStateMachine;
    }

    protected StateMachine createBallGrabberStateMachine() {
        StateMachine ballGrabberStateMachine = new StateMachine(telemetry);
        ballGrabberStateMachine.addSequential(new WaitForGrabBallCommandState(telemetry));
        ballGrabberStateMachine.addSequential(new BallGrabServoState(telemetry)); // tilt the forks
        ballGrabberStateMachine.addSequential(new DelayState("wait for the squeeze!", telemetry, 300, TimeUnit.MILLISECONDS));
        ballGrabberStateMachine.addSequential(new LiftSlightlyUpState(telemetry)); // run the lift
        ballGrabberStateMachine.addSequential(newDoneState("Done grabbing the ball"));

        return ballGrabberStateMachine;
    }

    class WaitForGrabBallCommandState extends State {
        public WaitForGrabBallCommandState(Telemetry telemetry) {
            super("Waiting to grab ball", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (liftSafety.isPressed()) {
                telemetry.addData("99", "Lift safety OFF");
                if (grabBallButton.getRise()) {
                    Log.d("VV", "Operator called for auto-grab");
                    return nextState;
                } else {
                    return this;
                }
            } else {
                telemetry.addData("99", "Lift safety ON");
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class LiftSlightlyUpState extends TimeoutSafetyState {
        boolean startedDrive = false;

        long encoderCountForLift = 0;

        public LiftSlightlyUpState(Telemetry telemetry) {
            super("Lifting cap ball", telemetry, 10000);
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        @Override
        public State doStuffAndGetNextState() {
            //if (!startedDrive) {
            //    liftDriveTrain.driveInches(-3, -1);
            //    startedDrive = true;

            //    return this;
            //}

            // fixme - use MR motor controllers for this, once we figure out why it's not working
            if (encoderCountForLift == 0) {
                encoderCountForLift = liftDriveTrain.getEncoderCountsForDriveInches(-3);
                Log.d("VV", "Required encoder count for lift = " + encoderCountForLift);
            }

            if (encoderCountForLift < 0) {
                liftMotor.setPower(-1);
            } else {
                liftMotor.setPower(1);
            }

            long currentEncoderCount = liftDriveTrain.getCurrentPosition();

            telemetry.addData("98", "Lift encoder position = " + currentEncoderCount);

            if (encoderCountForLift < 0) {
                if (currentEncoderCount <= encoderCountForLift) {
                    Log.d("VV", "Lift encoder count of " + currentEncoderCount + " <= " + encoderCountForLift);
                    liftMotor.setPower(0);
                    return nextState;
                }
            } else {
                if (currentEncoderCount >= encoderCountForLift) {
                    Log.d("VV", "Lift encoder count of " + currentEncoderCount + " >= " + encoderCountForLift);
                    liftMotor.setPower(0);
                    return nextState;
                }
            }


            if (isTimedOut()) {
                Log.d("VV", "Auto lifting timed out, stopping lift");
                resetLiftMotor();

                return nextState;
            }

            return this;
        }

        protected void resetLiftMotor() {
            //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
        }

        @Override
        public void resetToStart() {
            startedDrive = false;
        }

    }

    class BallGrabServoState extends State {

        public BallGrabServoState(Telemetry telemetry) {
            super("Ball grab servo squeeze", telemetry);
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        @Override
        public State doStuffAndGetNextState() {
            forkTiltServo.setPosition(0.07);
            return nextState;
        }

        @Override
        public void resetToStart() {
            tiltForkServoToStartingPosition();
        }
    }

    private void handleDrive() {
        cheesyDrive.handleDrive();
    }

    private void updateGamepadTelemetry() {
        telemetry.addData ("06", "GP1 Left x: " + -driverLeftStickX.getPosition());
        telemetry.addData ("07", "GP1 Left y: " + -driverLeftStickY.getPosition());
    }

}