/**
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
 **/

package com.hfrobots.tnt.season1920;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.AnyButton;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.LowPassFilteredRangeInput;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.ParametricScaledRangeInput;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.control.ToggledButton;

import lombok.Builder;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class DriverControls {
    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected DebouncedButton dpadUp;

    protected DebouncedButton dpadDown;

    protected DebouncedButton dpadLeft;

    protected DebouncedButton dpadRight;

    protected OnOffButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected DebouncedButton yYellowButton;

    protected DebouncedButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    protected OnOffButton useStationKeeping;

    // Derived
    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected OnOffButton driveInvertedButton;

    protected OnOffButton driveFastButton;

    protected OnOffButton driveBumpStrafeRightButton;

    protected OnOffButton driveBumpStrafeLeftButton;

    protected OnOffButton dpadUpRaw;

    protected OnOffButton dpadDownRaw;

    protected OnOffButton dpadLeftRaw;

    protected OnOffButton dpadRightRaw;

    private NinjaGamePad driversGamepad;

    private OpenLoopMecanumKinematics kinematics;

    private FoundationGripMechanism foundationGripMechanism;

    private StationKeeping stationKeeping;

    private ToggledButton parkingStickToggledButton;

    private ParkingSticks parkingSticks;

    protected DebouncedButton foundationGripButton;

    protected DebouncedButton foundationUngripButton;

    private final float throttleGain = 0.7F;

    private final float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    private final float throttleDeadband = 0;

    private final float lowPassFilterFactor = .95F;

    @Builder
    private DriverControls(RangeInput leftStickX,
                           RangeInput leftStickY,
                           RangeInput rightStickX,
                           RangeInput rightStickY,
                           DebouncedButton dpadUp,
                           DebouncedButton dpadDown,
                           DebouncedButton dpadLeft,
                           DebouncedButton dpadRight,
                           OnOffButton dpadUpRaw,
                           OnOffButton dpadDownRaw,
                           OnOffButton dpadLeftRaw,
                           OnOffButton dpadRightRaw,
                           OnOffButton xBlueButton,
                           DebouncedButton bRedButton,
                           DebouncedButton yYellowButton,
                           DebouncedButton aGreenButton,
                           OnOffButton rightBumper,
                           OnOffButton leftBumper,
                           RangeInput leftTrigger,
                           RangeInput rightTrigger,
                           NinjaGamePad driversGamepad,
                           OpenLoopMecanumKinematics kinematics,
                           FoundationGripMechanism foundationGripMechanism,
                           StationKeeping stationKeeping,
                           ParkingSticks parkingSticks) {
        if (driversGamepad != null) {
            this.driversGamepad = driversGamepad;
            setupFromGamepad();
        } else {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.rightStickY = rightStickY;
            this.dpadUp = dpadUp;
            this.dpadDown = dpadDown;
            this.dpadLeft = dpadLeft;
            this.dpadRight = dpadRight;
            this.xBlueButton = xBlueButton;
            this.bRedButton = bRedButton;
            this.yYellowButton = yYellowButton;
            this.aGreenButton = aGreenButton;
            this.rightBumper = rightBumper;
            this.leftBumper = leftBumper;
            this.leftTrigger = leftTrigger;
            this.rightTrigger = rightTrigger;
            this.dpadUpRaw = dpadUpRaw;
            this.dpadDownRaw = dpadDownRaw;
            this.dpadLeftRaw = dpadLeftRaw;
            this.dpadRightRaw = dpadRightRaw;
        }

        setupDerivedControls();
        setupCurvesAndFilters();

        this.kinematics = kinematics;
        this.foundationGripMechanism = foundationGripMechanism;
        this.stationKeeping = stationKeeping;
        this.parkingSticks = parkingSticks;
    }

    private void setupCurvesAndFilters() {
        driveStrafe = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(leftStickX, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveForwardReverse = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(leftStickY, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveRotate = new LowPassFilteredRangeInput(rightStickX, lowPassFilterFactor);
    }

    private void setupFromGamepad() {
        leftStickX = driversGamepad.getLeftStickX();
        leftStickY = driversGamepad.getLeftStickY();
        rightStickX = driversGamepad.getRightStickX();
        rightStickY = driversGamepad.getRightStickY();

        dpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        dpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        dpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        dpadRight = new DebouncedButton(driversGamepad.getDpadRight());

        dpadUpRaw = driversGamepad.getDpadUp();
        dpadDownRaw = driversGamepad.getDpadDown();
        dpadLeftRaw = driversGamepad.getDpadLeft();
        dpadRightRaw = driversGamepad.getDpadRight();

        aGreenButton = new DebouncedButton(driversGamepad.getAButton());
        bRedButton = new DebouncedButton(driversGamepad.getBButton());
        xBlueButton = driversGamepad.getXButton();
        yYellowButton = new DebouncedButton(driversGamepad.getYButton());
        leftBumper = driversGamepad.getLeftBumper();
        rightBumper = driversGamepad.getRightBumper();
        leftTrigger = driversGamepad.getLeftTrigger();
        rightTrigger = driversGamepad.getRightTrigger();

        // FIXME: Not yet mocked for tests
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
    }

    private void setupDerivedControls() {
        driveFastButton = new RangeInputButton(leftTrigger, 0.65f);
        driveInvertedButton = new RangeInputButton(rightTrigger, 0.65f);
        driveBumpStrafeLeftButton = leftBumper;
        driveBumpStrafeRightButton = rightBumper;
        foundationGripButton = aGreenButton;
        foundationUngripButton = bRedButton;
        useStationKeeping = xBlueButton;

        AnyButton anyDpadButton = new AnyButton(dpadUpRaw,dpadDownRaw,dpadLeftRaw,dpadRightRaw);
        parkingStickToggledButton = new ToggledButton(anyDpadButton);
    }

    private boolean gripUpFirstTime = false;

    public void periodicTask() {
        double x = driveStrafe.getPosition();
        double y = - driveForwardReverse.getPosition();
        double rot = driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis
        boolean useEncoders = false;

        // do this first, it will be cancelled out by bump-strafe
        if (!driveFastButton.isPressed()) {
            y /= 1.5;
            x /= 1.25;
            rot /= 1.5;
            useEncoders = false;
        }

        final boolean driveInverted;

        if (driveInvertedButton.isPressed()) {
            driveInverted = true;
        } else {
            driveInverted = false;
        }

        double xScaled = x;
        double yScaled = y;
        double rotateScaled = rot;

        // we check both bumpers - because both being pressed is driver 'panic', and we
        // don't want unexpected behavior!
        if (driveBumpStrafeLeftButton.isPressed() && !driveBumpStrafeRightButton.isPressed()) {
            xScaled = -.6;
            yScaled = 0;
            rotateScaled = 0;
        } else if (driveBumpStrafeRightButton.isPressed() && !driveBumpStrafeLeftButton.isPressed()) {
            xScaled = .6;
            yScaled = 0;
            rotateScaled = 0;
        }

        if ((xBlueButton != null && xBlueButton.isPressed()) && stationKeeping != null) {
            StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

            switch (signals.getState()) {
                case ACTIVE:
                case ON_STATION:
                    rotateScaled = signals.getTurnPower();
                    break;
                case TOO_CLOSE:
                case TOO_FAR:
                    break;

            }
        } else {

        }

        kinematics.driveCartesian(xScaled, yScaled, rotateScaled, driveInverted, 0.0, useEncoders);

        handleFoundationGripper();

        handleParkingSticks();
    }

    private void handleFoundationGripper() {
        if (foundationGripMechanism == null) {
            Log.e(LOG_TAG, "Missing foundation grip mechanism!");

            return;
        }

        // Tricky - tri-state, at init, inside 18" constraints, at tele-op start, deployed forward
        // to avoid lift coiled cable
        if (!gripUpFirstTime) {
            foundationGripMechanism.up();
            gripUpFirstTime = true;
        }

        if (foundationGripButton != null && foundationGripButton.getRise()) {
            foundationGripMechanism.down();
        } else if (foundationUngripButton != null && foundationUngripButton.getRise()) {
            foundationGripMechanism.up();
        }
    }

    private void handleParkingSticks() {
        if (parkingSticks != null) {
            if (parkingStickToggledButton.isToggledTrue()) {
                 parkingSticks.deploy();
            } else {
                parkingSticks.stow();
            }
        }
    }
}
