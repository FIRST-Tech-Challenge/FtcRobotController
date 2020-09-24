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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * State machine state that will drive a TankDrive until the line is sensed and the robot
 * has used two optical distance sensors to square up on the line.
 */
public class DriveUntilLineState extends TimeoutSafetyState {
    private final static double MIN_DRIVE_POWER = .18;

    // TODO lauren - what do we do with it?
    private double maxInboardGreyOdsLevel = Double.MIN_VALUE;

    private double maxOutboardGreyOdsLevel = Double.MIN_VALUE;

    private ModernRoboticsI2cGyro gyro;

    private final TankDrive drive;

    // TODO lauren - when do we get these, when do we check against max?
    private long inboardEncoderCountsStartValueWhileSeeking = Integer.MIN_VALUE;

    private long outboardEncoderCountsStartValueWhileSeeking = Integer.MIN_VALUE;

    // TODO lauren - how/when do we compute this?
    private long maxEncoderCountsWhileSeeking = 0;

    private final double powerLevel;

    private final OpticalDistanceSensor inboardLineSensor;

    private final OpticalDistanceSensor outboardLineSensor;

    private boolean driveStarted = false;

    // Are we in a state where we've found the line on at least one side, but not the other?
    private boolean squaringUp = false;

    // We use this to determine if we're hunting...braking inboard, then braking outboard, count
    // the cycles, die if we've done too many
    private boolean brakingInboard = false;

    private int cycleCount = 0;

    private final int MAX_CYCLE_COUNT = 5;

    private long startedOscillatingTimeMs = 0;

    /**
     * Constructs a state machine state that will drive the TankDrive (- is reverse)
     * at the given power level until the white line detected. The robot will
     * then attempt to square up on the line using inboard/outboard optical distance sensors.
     *
     * The state will transition to the state given by setNextState() when the line has been detected
     * and the robot has squared up, or if the timeout is reached.
     */
    public DriveUntilLineState(String name, TankDrive drive,
                               Telemetry telemetry,
                               OpticalDistanceSensor inboardLineSensor,
                               OpticalDistanceSensor outboardLineSensor,
                               ModernRoboticsI2cGyro gyro,
                               double powerLevel,
                               long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.powerLevel = powerLevel;
        this.outboardLineSensor = outboardLineSensor;
        this.inboardLineSensor = inboardLineSensor;
        this.gyro = gyro;
        maxEncoderCountsWhileSeeking = drive.getEncoderCountsForInchesTravel(1.7);
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!driveStarted) {
            Log.i("VV", "Drive until line - drive started");
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.drivePower(powerLevel, powerLevel);
            driveStarted = true;

            maxInboardGreyOdsLevel = inboardLineSensor.getRawLightDetected();

            maxOutboardGreyOdsLevel = outboardLineSensor.getRawLightDetected();

            Log.d("VV", "Grey ODS levels inboard/outboard " + maxInboardGreyOdsLevel + " / " + maxOutboardGreyOdsLevel);

            return this;
        }

        // FIXME this is only because coach mark is interested in this data for now
        //if (gyro != null) {
        //    Log.d("VV", "squaring up, gyro heading is " + gyro.getIntegratedZValue());
        //}

        // left side of tank drive is "inboard"

        // ODS threshold
        double inboardThreshold = maxInboardGreyOdsLevel + .5;
        double outboardThreshold = maxOutboardGreyOdsLevel + .5;

        double inboardLightDetected = inboardLineSensor.getRawLightDetected();
        double outboardLightDetected = outboardLineSensor.getRawLightDetected();

        if (inboardLightDetected > inboardThreshold &&
                outboardLightDetected > outboardThreshold) {
            Log.i("VV", "Drive until line - line detected pressed- stopping drive");
            stopDriving();

            return nextState;
        } else {
            double squareUpPower = Math.max(MIN_DRIVE_POWER, powerLevel / 4);

            if (inboardLightDetected > inboardThreshold && outboardLightDetected <= inboardThreshold) {
                if (startedOscillatingTimeMs == 0) {
                    startedOscillatingTimeMs = System.currentTimeMillis();
                }

                maybeInitEncoders();

                TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();

                // stopping the robot from overshooting the line

                if (Math.abs(inboardEncoderCountsStartValueWhileSeeking - currentPositions.getLeftTargetPosition()) > maxEncoderCountsWhileSeeking
                        || Math.abs(outboardEncoderCountsStartValueWhileSeeking - currentPositions.getRightTargetPosition()) > maxEncoderCountsWhileSeeking) {
                    Log.d("VV", "Drove too far while attempting to detect line");
                    stopDriving();

                    return nextState;
                }

                if (!brakingInboard) {
                    cycleCount++;

                    if (cycleCount > MAX_CYCLE_COUNT) {
                        stopDriving();
                        Log.d("VV", "reached max cycle count");
                        return nextState;
                    }
                }

                brakingInboard = true;
                drive.drivePower(0 /* inboard */, squareUpPower /* outboard */);
                Log.d("VV", String.format("Found line inboard at level %f, outboard at %f, brake inboard, drive outboard",
                        inboardLightDetected, outboardLightDetected));
            } else if (outboardLightDetected > outboardThreshold && inboardLightDetected <= inboardThreshold) {
                if (startedOscillatingTimeMs == 0) {
                    startedOscillatingTimeMs = System.currentTimeMillis();
                }

                maybeInitEncoders();

                TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();

                // stopping the robot from overshooting the line

                if (Math.abs(inboardEncoderCountsStartValueWhileSeeking - currentPositions.getLeftTargetPosition()) > maxEncoderCountsWhileSeeking
                        || Math.abs(outboardEncoderCountsStartValueWhileSeeking - currentPositions.getRightTargetPosition()) > maxEncoderCountsWhileSeeking) {
                    Log.d("VV", "Drove too far while attempting to detect line");
                    stopDriving();

                    return nextState;
                }

                if (brakingInboard) {
                    cycleCount++;
                    if (cycleCount > MAX_CYCLE_COUNT) {
                        stopDriving();
                        Log.d("VV", "reached max cycle count");
                        return nextState;
                    }
                }

                brakingInboard = false;
                drive.drivePower(squareUpPower /*inboard*/, 0 /*outboard*/);
                Log.d("VV",
                        String.format("Found line outboard at level %f, inboard at %f, brake outboard, drive inboard",
                                outboardLightDetected, inboardLightDetected));
            }
        }

        if (startedOscillatingTimeMs != 0) {
            long oscillatingElapsedMs = System.currentTimeMillis() - startedOscillatingTimeMs;

            if (oscillatingElapsedMs > 3000){
                stopDriving();
                Log.d("VV", "Oscillating time limit reached");
                return nextState;
            }
        }

        if (isTimedOut()) {
            Log.e("VV", "Drive inches state timed out- stopping drive");
            stopDriving();

            return nextState;
        }

        return this;
    }

    protected void maybeInitEncoders() {
        if (inboardEncoderCountsStartValueWhileSeeking == Integer.MIN_VALUE &&
                outboardEncoderCountsStartValueWhileSeeking == Integer.MIN_VALUE) {
            // initialize them
            TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();
            inboardEncoderCountsStartValueWhileSeeking = currentPositions.getLeftTargetPosition();
            outboardEncoderCountsStartValueWhileSeeking = currentPositions.getRightTargetPosition();
        }
    }

    @Override
    public void resetToStart() {
        super.resetToStart();

        driveStarted = false;

        brakingInboard = false;

        cycleCount = 0;

        startedOscillatingTimeMs = 0;
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }

    private void stopDriving() {
        // (1) Stop the motors
        drive.drivePower(0, 0);
        // (2) "reset" the motors/drive train to a "normal" state, which is?
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
