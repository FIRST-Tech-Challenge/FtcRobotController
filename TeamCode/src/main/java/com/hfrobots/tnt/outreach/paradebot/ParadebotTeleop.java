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

package com.hfrobots.tnt.outreach.paradebot;


import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.NewCheesyDrive;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Iterator;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@TeleOp(name="Paradebot Teleop")
@Disabled
public class ParadebotTeleop extends OpMode

{
    protected ParadebotRobot robot;

    protected float throttleGain = 0.7F;

    protected float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    protected float throttleDeadband = 0;

    protected NinjaGamePad driversGamepad;

    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected RangeInput driveForwardReverse;

    protected RangeInput driveRotate;

    protected NewCheesyDrive cheesyDrive;

    protected VoltageSensor voltageSensor;

    protected DebouncedButton cruizeControlStart;

    protected DebouncedButton cruizeControlStop;

    protected DebouncedButton cruizeControlAccelerate;

    protected DebouncedButton cruizeControlDeccelerate;

    protected boolean isTomCruizing = false;

    protected double cruizeThottle;

    private double flagPosition;

    private double flagPositionLow = 0;

    private double flagPositionHigh = .35;

    private double flagPositionDelta = (flagPositionHigh - flagPositionLow) / 300;

    private double flagDirection = 1;

    private DebouncedButton flagbutton;

    private boolean isFlagWaving = false;

    @SuppressWarnings("unused")
    public ParadebotTeleop() {
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        Servo flagWaverServo = hardwareMap.servo.get("flagWaverServo");


        robot = new ParadebotRobot(
                NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("leftDriveMotor")),
                NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("rightDriveMotor")),
                flagWaverServo
        );

        cheesyDrive = new NewCheesyDrive();

        setupDriverControls();

        Iterator<VoltageSensor> voltageSensors = hardwareMap.voltageSensor.iterator();

        if (voltageSensors.hasNext()) {
            voltageSensor = voltageSensors.next();
        }
    }


    private final float lowPassFilterFactor = .92F;

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();
        driverRightStickX = driversGamepad.getRightStickX();
        driverRightStickY = driversGamepad.getRightStickY();

        driveForwardReverse = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(driverLeftStickY, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveRotate = new LowPassFilteredRangeInput(driverRightStickX, lowPassFilterFactor);
        flagbutton = new DebouncedButton(driversGamepad.getXButton());

        cruizeControlStart = new DebouncedButton(driversGamepad.getAButton());
        cruizeControlStop = new DebouncedButton(driversGamepad.getBButton());
        cruizeControlAccelerate = new DebouncedButton(driversGamepad.getDpadUp());
        cruizeControlDeccelerate = new DebouncedButton(driversGamepad.getDpadDown());

    }

    @Override public void loop () {
        handleDrivingInputs();

        handleFlagWaver();
    }

    private void handleFlagWaver() {
        if (flagbutton.getRise()) {
            isFlagWaving = !isFlagWaving;
        }

        if (isFlagWaving) {
            flagPosition += flagDirection * flagPositionDelta;

            robot.setFlagPosition(flagPosition);

            if (flagPosition < flagPositionLow) {
                flagDirection = 1;
            } else if (flagPosition > flagPositionHigh) {
                flagDirection = -1;
            }
        } else {
            robot.setFlagPosition(.15);
        }
    }

    @Override
    public void stop() {
        Log.d(LOG_TAG, "x-throttle-usage: " + xThrottleHistogram.toString());
        Log.d(LOG_TAG, "y-throttle-usage: " + yThrottleHistogram.toString());
        Log.d(LOG_TAG, "rot-throttle-usage: " + rotateThrottleHistogram.toString());
        super.stop();
    }

    private void handleDrivingInputs() {

        if (driversGamepad.getLeftTrigger().getPosition() >= .75 &&
                driversGamepad.getRightTrigger().getPosition() >= .75) {
            // Check for "panic" stop, cancel cruise control, stop the motion of the robot
            robot.hardStop();
            isTomCruizing= false;
            //we can't have him jumping on the couches.
        } else {
            // Get driver inputs
            double power = -driveForwardReverse.getPosition();
            double steer = driveRotate.getPosition();
            boolean isQuickTurn = driversGamepad.getLeftBumper().isPressed() ||
                    driversGamepad.getRightBumper().isPressed();

            // Are we enabling or disabling "cruise control"?

            if (cruizeControlStart.getRise() && power > 0) {
                isTomCruizing = true;
                cruizeThottle = power;
            }

            if (cruizeControlStop.getRise()) {
                isTomCruizing = false;
            }

            if (isTomCruizing == true) {
                if (cruizeControlAccelerate.getRise()) {
                    cruizeThottle += .05;
                }

                if (cruizeControlDeccelerate.getRise())  {
                    cruizeThottle -= .05;

                }

                cruizeThottle = Range.clip(cruizeThottle, 0, 1);

                power =  cruizeThottle;
            }

            // Use NewCheesyDrive to calculate left/right power values
            NewCheesyDrive.DriveSignal computedPower = cheesyDrive.cheesyDrive(
                    power, steer, isQuickTurn, true /* go fast parade bot! */);

            // Tell the robot to drive using those values
            robot.drive(computedPower.getLeft(), computedPower.getRight());
        }
            //
            // xThrottleHistogram.accumulate(xScaled);
            // yThrottleHistogram.accumulate(yScaled);
            // rotateThrottleHistogram.accumulate(rotateScaled);

        //telemetry.addData("pow", "y %.3f, x %.3f, r %.3f", yScaled, xScaled, rotateScaled);
    }

    private Histogram xThrottleHistogram = new Histogram();
    private Histogram yThrottleHistogram = new Histogram();
    private Histogram rotateThrottleHistogram = new Histogram();

    class Histogram {
        int[] buckets = new int[20];

        void accumulate(double value) {
            int bucketIndex = Range.clip((int)(Math.abs(value) * buckets.length), 0, buckets.length - 1);
            buckets[bucketIndex]++;
        }

        @Override
        public String toString() {
            StringBuffer buf = new StringBuffer();
            for (int i = 0; i < buckets.length; i++) {
                if (i != 0) {
                    buf.append(", ");
                }
                buf.append(buckets[i]);
            }

            return buf.toString();
        }
    }

    protected void logBatteryState(String opModeMethod) {
        if (voltageSensor == null) {
            Log.e("VV", String.format("No voltage sensor when logging voltage for %s"));

            return;
        }

        Log.d("VV", String.format("Robot battery voltage %5.2f at method %s()",voltageSensor.getVoltage(), opModeMethod));
    }
}

