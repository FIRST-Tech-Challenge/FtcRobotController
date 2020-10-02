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

package com.hfrobots.tnt.util.jackiebot;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Iterator;
import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public abstract class JackiebotHardware extends OpMode {
    protected float throttleGain = 0.7F;
    protected float throttleExponent = 5; // MUST BE AN ODD NUMBER!
    protected float throttleDeadband = 0;

    protected boolean imuNeeded = true;

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean warningGenerated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String warningMessage;

    protected NinjaGamePad driversGamepad;

    protected NinjaGamePad operatorsGamepad;

    // Drivebase
    protected ExtendedDcMotor leftFrontDriveMotor;

    protected ExtendedDcMotor rightFrontDriveMotor;

    protected ExtendedDcMotor leftRearDriveMotor;

    protected ExtendedDcMotor rightRearDriveMotor;

    protected MecanumDrive mecanumDrive;

    protected LynxEmbeddedIMU imu;

    protected VoltageSensor voltageSensor;

    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton driverDpadUp;

    protected DebouncedButton driverDpadDown;

    protected DebouncedButton driverDpadLeft;

    protected DebouncedButton driverDpadRight;

    protected DebouncedButton driverXBlueButton;

    protected DebouncedButton driverBRedButton;

    protected DebouncedButton driverYYellowButton;

    protected DebouncedButton driverAGreenButton;

    protected DebouncedButton driverRightBumper;

    protected DebouncedButton driverLeftBumper;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected OnOffButton driveInvertedButton;

    protected OnOffButton driveFastButton;

    protected OnOffButton driveBumpStrafeRightButton;

    protected OnOffButton driveBumpStrafeLeftButton;


    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * <p/>
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {
        setupDriverControls();

        setupOperatorControls();

        setupDrivebase();

        if (imuNeeded) {
            initImu();
        } else {
            Log.d(LOG_TAG, "IMU not required, not initializing");
        }

        Iterator<VoltageSensor> voltageSensors = hardwareMap.voltageSensor.iterator();

        if (voltageSensors.hasNext()) {
            voltageSensor = voltageSensors.next();
        }
    }

    protected void setupDrivebase() {
        try {
            leftFrontDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("leftFrontDriveMotor"));
            leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ex) {
            appendWarningMessage("leftFrontDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            leftFrontDriveMotor = null;
        }

        try {
            leftRearDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("leftRearDriveMotor"));
            leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ex) {
            appendWarningMessage("leftRearDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            leftRearDriveMotor = null;
        }

        try {
            rightFrontDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("rightFrontDriveMotor"));
            rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("rightFrontDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            rightFrontDriveMotor = null;
        }

        try {
            rightRearDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("rightRearDriveMotor"));
            rightRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("rightRearDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            rightRearDriveMotor = null;
        }

        mecanumDrive = MecanumDrive.builder().leftFrontDriveMotor(leftFrontDriveMotor)
                .rightFrontDriveMotor(rightFrontDriveMotor)
                .leftRearDriveMotor(leftRearDriveMotor)
                .rightRearDriveMotor(rightRearDriveMotor).build();
    }

    protected void initImu() {
        try {
            imu = hardwareMap.get(LynxEmbeddedIMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode                = BNO055IMU.SensorMode.IMU; // yes, it's the default, but let's be sure
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = false;
            parameters.loggingTag          = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();
            imu.initialize(parameters);
            imu.startAccelerationIntegration(null, null, 50); // not started by default?
        } catch (Exception ex) {
            appendWarningMessage("imu");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            imu = null;
        }
    }


    /**
     * Access whether a warning has been generated.
     */
    boolean wasWarningGenerated() {
        return warningGenerated;
    }

    /**
     * Access the warning message.
     */
    String getWarningMessage()

    {
        return warningMessage;
    }

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void appendWarningMessage(String exceptionMessage) {
        if (warningGenerated) {
            warningMessage += ", ";
        }
        warningGenerated = true;
        warningMessage += exceptionMessage;
    }

    protected void logBatteryState(String opModeMethod) {
        if (voltageSensor == null) {
            Log.e("VV", String.format("No voltage sensor when logging voltage for %s"));

            return;
        }

        Log.d("VV", String.format("Robot battery voltage %5.2f at method %s()",voltageSensor.getVoltage(), opModeMethod));
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    protected State newDoneState(String name) {
        return new State(name, telemetry) {
            private boolean issuedStop = false;

            @Override
            public State doStuffAndGetNextState() {
                if (!issuedStop) {
                    mecanumDrive.stopAllDriveMotors();

                    issuedStop = true;
                }

                return this;
            }

            @Override
            public void resetToStart() {
                issuedStop = false;
            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };
    }

    protected State newDelayState(String name, final int numberOfSeconds) {
        return new State(name, telemetry) {

            private long startTime = 0;
            private long thresholdTimeMs = TimeUnit.SECONDS.toMillis(numberOfSeconds);

            @Override
            public void resetToStart() {
                startTime = 0;
            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }

            @Override
            public State doStuffAndGetNextState() {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                    return this;
                }

                long now = System.currentTimeMillis();
                long elapsedMs = now - startTime;

                if (elapsedMs > thresholdTimeMs) {
                    return nextState;
                }

                telemetry.addData("04", "Delay: %d of %d ms", elapsedMs, thresholdTimeMs);
                return this;
            }
        };
    }

    private void setupOperatorControls() {
        // Operator controls
        operatorsGamepad = new NinjaGamePad(gamepad2);
    }

    private final float lowPassFilterFactor = .95F; // Fixme may not be right value...

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();
        driverRightStickX = driversGamepad.getRightStickX();
        driverRightStickY = driversGamepad.getRightStickY();

        driveStrafe = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(driverLeftStickX, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveForwardReverse = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(driverLeftStickY, lowPassFilterFactor),
            throttleDeadband, throttleGain, throttleExponent);

        driveRotate = new LowPassFilteredRangeInput(driverRightStickX, lowPassFilterFactor);

        driverDpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        driverDpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        driverDpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        driverDpadRight = new DebouncedButton(driversGamepad.getDpadRight());
        driverAGreenButton = new DebouncedButton(driversGamepad.getAButton());
        driverBRedButton = new DebouncedButton(driversGamepad.getBButton());
        driverXBlueButton = new DebouncedButton(driversGamepad.getXButton());
        driverYYellowButton = new DebouncedButton(driversGamepad.getYButton());
        driverLeftBumper = new DebouncedButton(driversGamepad.getLeftBumper());
        driverRightBumper = new DebouncedButton(driversGamepad.getRightBumper());
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
        driveFastButton = new RangeInputButton(driversGamepad.getLeftTrigger(), 0.65f);
        driveInvertedButton = new RangeInputButton(driversGamepad.getRightTrigger(), 0.65f);
        driveBumpStrafeLeftButton = driversGamepad.getLeftBumper();
        driveBumpStrafeRightButton = driversGamepad.getRightBumper();
    }
}
