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

package com.hfrobots.tnt.season1516;


import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Provide a basic manual operational mode that controls the holonomic drive.
 */
@TeleOp(name="SkittleBot Teleop")
@Disabled
public class SkittleBotTeleop extends SkittleBotTelemetry

{
    protected boolean useEncoders = false;

    /*
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public SkittleBotTeleop() {
    }

    private NinjaGamePad driverGamePad;

    private RangeInput leftX;

    private LowPassFilteredRangeInput leftXFilter;

    private ParametricScaledRangeInput leftXScaled;

    private RangeInput leftY;

    private LowPassFilteredRangeInput leftYFilter;

    private ParametricScaledRangeInput leftYScaled;

    private RangeInput rightX;

    private LowPassFilteredRangeInput rightXFilter;

    private ParametricScaledRangeInput rightXScaled;

    private RangeInput rightY;

    private LowPassFilteredRangeInput rightYFilter;

    private ParametricScaledRangeInput rightYScaled;

    private DebouncedButton lowPassIncrease;

    private DebouncedButton lowPassDecrease;

    private DebouncedButton curveExponentIncrease;

    private DebouncedButton curveExponentDecrease;

    private DebouncedButton curveGainIncrease;

    private DebouncedButton curveGainDecrease;

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        super.init();

        if (!useEncoders) {
            runWithoutDriveEncoders();
        } else {
            runUsingEncoders();
        }

        //runWithoutBraking();

        driverGamePad = new NinjaGamePad(gamepad1);

        lowPassDecrease = new DebouncedButton(driverGamePad.getDpadDown());
        lowPassIncrease = new DebouncedButton(driverGamePad.getDpadUp());
        curveExponentIncrease = new DebouncedButton(driverGamePad.getDpadRight());
        curveExponentDecrease = new DebouncedButton(driverGamePad.getDpadLeft());
        curveGainIncrease = new DebouncedButton(driverGamePad.getRightBumper());
        curveGainDecrease = new DebouncedButton(driverGamePad.getLeftBumper());

        setupFiltersAndCurves();

        // set servos to initial positions
        setClimberDumpServoPosition(.5); // sets CR servo to stop position
        setWinchAimServoPosition(.5); // sets CR servo to stop position
    }

    private float lowPassFilterFactor = 1.0F;

    protected float throttleGain = 0F;
    protected float throttleExponent = 5; // MUST BE AN ODD NUMBER!
    protected float throttleDeadband = 0;

    private void setupFiltersAndCurves() {
        leftXFilter = new LowPassFilteredRangeInput(driverGamePad.getLeftStickX(), lowPassFilterFactor);
        leftXScaled = new ParametricScaledRangeInput(leftXFilter, throttleDeadband, throttleGain, throttleExponent);
        leftX = leftXFilter;

        leftYFilter = new LowPassFilteredRangeInput(driverGamePad.getLeftStickY(), lowPassFilterFactor);
        leftYScaled = new ParametricScaledRangeInput(leftYFilter, throttleDeadband, throttleGain, throttleExponent);
        leftY = leftYFilter;

        rightXFilter = new LowPassFilteredRangeInput(driverGamePad.getRightStickX(), lowPassFilterFactor);
        rightXScaled = new ParametricScaledRangeInput(rightXFilter, throttleDeadband, throttleGain, throttleExponent);
        rightX = rightXScaled;

        rightYFilter = new LowPassFilteredRangeInput(driverGamePad.getRightStickY(), lowPassFilterFactor);
        rightYScaled = new ParametricScaledRangeInput(rightYFilter, throttleDeadband, throttleGain, throttleExponent);
        rightY = rightYScaled;
    }


    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {


        steeringPriorityDrive();

        // The climber dump servo is continuous rotation, 0.5 is stop,
        // 0 is one direction, 1 the other
        /*
        if (gamepad2.dpad_down) {
            setClimberDumpServoPosition(1.0);
        } else if (gamepad2.dpad_up) {
            setClimberDumpServoPosition(0);
        } else {
            setClimberDumpServoPosition(0.5);
        }

        handleWinchControls();

        colorSensorRecon();
        */

        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData("01", "T_G: %.1f T_EXP: %.0f LP: %.2f", throttleGain, throttleExponent, lowPassFilterFactor);
        updateTelemetry(); // Update common telemetry
    }

    private void colorSensorRecon() {
        if (gamepad2.left_trigger > 0.5 || gamepad2.left_bumper) {
            ColorSensorValues reading = getColorSensorValues();
            Log.d(LOG_TAG, "Color sensor reading: " + reading.toString());
        }
    }

    private void steeringPriorityDrive() {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until the loop() method ends.
        //

        handleParamterChanges();

        float spinControl = -rightX.getPosition();

        final float yDrivePower;
        final float xDrivePower;

        if (Math.abs(spinControl) > 0) {
            yDrivePower = xDrivePower = spinControl;
            setDrivePower(xDrivePower, xDrivePower, yDrivePower, yDrivePower);
        } else {
            float leftStickY = -leftY.getPosition();
            float leftStickX = -leftX.getPosition();

            double orientationShiftDegrees = getOrientationShiftDegrees();

            if (gamepad1.right_trigger < .65) {
                double fortyFiveDegRad = Math.toRadians(-45.0D + orientationShiftDegrees);
                double leftStickXPrime = leftStickX * Math.cos(fortyFiveDegRad) - leftStickY * Math.sin(fortyFiveDegRad);
                double leftStickYPrime = leftStickX * Math.sin(fortyFiveDegRad) + leftStickY * Math.cos(fortyFiveDegRad);

                leftStickX = (float)leftStickXPrime;
                leftStickY = (float)leftStickYPrime;
            }

            yDrivePower = leftStickY;
            xDrivePower = leftStickX;

            setDrivePower(xDrivePower, -xDrivePower, yDrivePower, -yDrivePower);
        }
    }

    private void handleParamterChanges() {
        boolean parametersChanged = false;

        if (lowPassDecrease.getRise()) {
            lowPassFilterFactor -= .05;
            if (lowPassFilterFactor < .05) {
                lowPassFilterFactor = .05F;
            }
            parametersChanged = true;
        } else if (lowPassIncrease.getRise()) {
            lowPassFilterFactor += .05;

            if (lowPassFilterFactor > 1) {
                lowPassFilterFactor = 1.0F;
            }

            parametersChanged = true;
        }

        if (curveExponentDecrease.getRise()) {
            throttleExponent -= 2;
            if (throttleExponent < 1){
                throttleExponent = 1;
            }
            parametersChanged = true;
        } else if (curveExponentIncrease.getRise()) {
            throttleExponent += 2;
            if (throttleExponent > 15) {
                throttleExponent = 15;
            }
            parametersChanged = true;
        }

        if (curveGainDecrease.getRise()) {
            throttleGain -= .1;

            if (throttleGain < 0) {
                throttleGain = 0;
            }

            parametersChanged = true;
        } else if (curveGainIncrease.getRise()) {
            throttleGain += .1;

            if (throttleGain > 1) {
                throttleGain = 1;
            }

            parametersChanged = true;
        }

        if (parametersChanged) {
            setupFiltersAndCurves();
        }
    }

    private void simultaneousSteerDrive() {
        float spinControl = -gamepad1.right_stick_x;

        final float yDriveOnlyPower;
        final float xDriveOnlyPower;
        float spinPower = 0;

        if (Math.abs(spinControl) > 0) {
            spinPower = scaleMotorPower(spinControl);
        }

        spinPower = 0;
        float leftStickY = -gamepad1.left_stick_y;
        float leftStickX = -gamepad1.left_stick_x;

        double orientationShiftDegrees = getOrientationShiftDegrees();

        if (gamepad1.right_trigger < .65) {
            double fortyFiveDegRad = Math.toRadians(-45.0D + orientationShiftDegrees);
            double leftStickXPrime = leftStickX * Math.cos(fortyFiveDegRad) - leftStickY * Math.sin(fortyFiveDegRad);
            double leftStickYPrime = leftStickX * Math.sin(fortyFiveDegRad) + leftStickY * Math.cos(fortyFiveDegRad);

            leftStickX = (float)leftStickXPrime;
            leftStickY = (float)leftStickYPrime;
        }

        yDriveOnlyPower = scaleMotorPower(leftStickY);
        xDriveOnlyPower = scaleMotorPower(leftStickX);

        float x1DrivePower = Range.clip(xDriveOnlyPower + spinPower, -1, 1);
        float x2DrivePower = Range.clip(-xDriveOnlyPower + spinPower,-1,1);
        float y1DrivePower = Range.clip(yDriveOnlyPower + spinPower, -1,1);
        float y2DrivePower = Range.clip(-yDriveOnlyPower + spinPower,-1,1);

        setDrivePower(x1DrivePower, x2DrivePower, y1DrivePower, y2DrivePower);
    }

    private void handleWinchControls() {
        float winchPower = gamepad2.left_stick_y;
        float aimValue = gamepad2.right_stick_y;

        setWinchDrivePower(-scaleMotorPower(winchPower));

        if (aimValue > 0) {
            setWinchAimServoPosition(1);
        } else if (aimValue < 0) {
            setWinchAimServoPosition(0);
        } else {
            setWinchAimServoPosition(0.5); // stop
        }
    }

    /**
     * Shifts the "front" of the robot to one of the sides, mapped to
     * the button layout on the driver's controller. The shift is only
     * in effect while the button is depressed, and returns to the normal
     * front side when the button is released.
     *
     *       Y
     *  X         B
     *       A
     */
    private double getOrientationShiftDegrees() {
        if (gamepad1.y) {
            return 0; /* we have it at zero because this is the starting point*/
        } else if (gamepad1.b) {
            return -90; /* negative because of right-hand rule, z-axis points up on our robot */
        } else if (gamepad1.a) {
            return -180;
        } else if (gamepad1.x) {
            return -270;
        } else {
             return 0; /* we want it to revert back to it original state if no button is pressed */
        }
    }
}
