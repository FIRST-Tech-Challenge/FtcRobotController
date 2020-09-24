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

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.LowPassFilteredRangeInput;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.ParametricScaledRangeInput;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Iterator;
import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public abstract class RoverRuckusHardware extends OpMode {
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

    protected DcMotor acDcMotor;

    protected DcMotor collectorDeployMotor;

    protected DcMotor collectorSweepMotor;

    protected RangeInput collectorDeployInput;

    protected RangeInput collectorSweepInput;

    protected DigitalChannel acDcLimitSwitch;

    protected DigitalChannel collectorStowLimitSwitch;

    protected DigitalChannel scoreBoxReadyToLoadLimitSwitch;

    protected LinearActuator ascenderDescender;

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

    protected OnOffButton acDcExtendButton;

    protected OnOffButton acDcRetractButton;

    protected DebouncedButton acDcHomeButton;

    protected OnOffButton acDcLimitOverrideButton;

    protected DebouncedButton acDcStopButton;

    protected OnOffButton elevatorCommandUpButton;

    protected OnOffButton elevatorCommandDownButton;

    protected DebouncedButton elevatorUpperLimitButton;

    protected DebouncedButton elevatorLowerLimitButton;

    protected DebouncedButton elevatorEmergencyStopButton;

    protected OnOffButton loadToBoxButton;

    protected OnOffButton boxTipButton;

    protected ExtendedDcMotor particleScoreElevatorMotor;

    protected Servo boxTipServo;

    protected Servo boxGateServo;

    protected Servo collectorGateServo;

    protected Servo teamMarkerServo;

    protected ParticleScoringMechanism particleScoringMechanism;

    protected OnOffButton limitOverrideButton;

    protected Servo m3ArmServo;

    protected Servo m3FlagServo;

    /*
     * Perform any actions that are necessary when the OpMode is enabled.
     * <p/>
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {
        setupDriverControls();

        setupOperatorControls();

        setupDrivebase();

        setupAscenderDescender();

        setupCollector();

        setupParticleScoringMechanism();

        setupTeamMarker();

        setupM3();

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

    protected static double M3_ARM_OUT_POSITION = 0;

    protected static double M3_ARM_UP_POSITION = .49;

    protected static double M3_FLAG_STOWED_POSITION = .5111;

    protected static double M3_FLAG_DEPLOYED_POSITION = .891;

    protected void setupM3() {
        // arm servo - out_position = 0
        // arm servo - up-position=.49
        // Flag servo stowed=.494
        // Flag servo deployed=.891

        m3ArmServo = hardwareMap.get(Servo.class, "m3ArmServo");

        m3FlagServo = hardwareMap.get(Servo.class, "m3FlagServo");

        m3FlagUp();

        try {
            Thread.currentThread().sleep(500);
        } catch (InterruptedException intEx) {
            // do nothing
        }

        m3ArmUp();
    }

    protected void m3ArmUp() {
        m3ArmServo.setPosition(M3_ARM_UP_POSITION);
    }

    protected void m3ArmOut() { m3ArmServo.setPosition(M3_ARM_OUT_POSITION);}

    protected void m3FlagUp() {
        m3FlagServo.setPosition(M3_FLAG_STOWED_POSITION);
    }

    protected void m3FlagOut() { m3FlagServo.setPosition(M3_FLAG_DEPLOYED_POSITION);}

    protected void setupParticleScoringMechanism() {
        try {
            particleScoreElevatorMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("particleScoreElevatorMotor"));
            particleScoreElevatorMotor.setDirection(DcMotor.Direction.REVERSE);

            // Need to disable RUN_USING_ENCODER end ZPB.Brake to use a voltage feed forward for holding
            // but WHY?
            particleScoreElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            particleScoreElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception ex) {
            appendWarningMessage("particleScoreElevatorMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            particleScoreElevatorMotor = null;
        }
        try {
            scoreBoxReadyToLoadLimitSwitch = hardwareMap.digitalChannel.get("scoreBoxReadyToLoadLimitSwitch");
        } catch (Exception ex) {
            appendWarningMessage("scoreBoxReadyToLoadLimitSwitch");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            scoreBoxReadyToLoadLimitSwitch = null;
        }

        boxTipServo = hardwareMap.servo.get("boxTipServo");

        boxGateServo = hardwareMap.servo.get("boxGateServo");

        particleScoringMechanism = new ParticleScoringMechanism(elevatorCommandUpButton,
                elevatorCommandDownButton, elevatorUpperLimitButton, elevatorLowerLimitButton,
                elevatorEmergencyStopButton, particleScoreElevatorMotor, null, null,
                telemetry, boxTipServo, boxTipButton, limitOverrideButton);
    }

    public static final double TEAM_MARKER_DUMP_POS = 1.0; // .25;

    public static final double TEAM_MARKER_STOWED_STATE = 0; // 1.0;

    protected void setupTeamMarker() {
        teamMarkerServo = hardwareMap.servo.get("teamMarkerServo");
        teamMarkerServo.setPosition(TEAM_MARKER_STOWED_STATE);
    }


    protected void setupCollector() {
        try {
            collectorDeployMotor = hardwareMap.dcMotor.get("collectorDeployMotor");
            collectorDeployMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            collectorDeployMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            collectorDeployMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // MM
        } catch (Exception ex) {
            appendWarningMessage("collectorDeployMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            collectorDeployMotor = null;
        }

        try {
            collectorSweepMotor = hardwareMap.dcMotor.get("collectorSweepMotor");
        } catch (Exception ex) {
            appendWarningMessage("collectorSweepMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            collectorSweepMotor = null;
        }
        try {
            collectorStowLimitSwitch = hardwareMap.digitalChannel.get("collectorStowLimitSwitch");
        } catch (Exception ex) {
            appendWarningMessage("collectorStowLimitSwitch");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            collectorStowLimitSwitch = null;
        }

        collectorGateServo = hardwareMap.servo.get("collectorGateServo");
    }


    protected void setupDrivebase() {
        try {
            leftFrontDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("leftFrontDriveMotor"));
            leftFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("leftFrontDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            leftFrontDriveMotor = null;
        }

        try {
            leftRearDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("leftRearDriveMotor"));
            leftRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("leftRearDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            leftRearDriveMotor = null;
        }

        try {
            rightFrontDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("rightFrontDriveMotor"));
            rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ex) {
            appendWarningMessage("rightFrontDriveMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            rightFrontDriveMotor = null;
        }

        try {
            rightRearDriveMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("rightRearDriveMotor"));
            rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
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


    protected void setupAscenderDescender() {
        try {
            acDcMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.dcMotor.get("acDcMotor"));
            acDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception ex) {
            appendWarningMessage("acDcMotor");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            acDcMotor = null;
        }

        try {
            acDcLimitSwitch = hardwareMap.digitalChannel.get("acDcLimitSwitch");
        } catch (Exception ex) {
            appendWarningMessage("acDcLimitSwitch");
            Log.e(LOG_TAG, ex.getLocalizedMessage());

            acDcLimitSwitch = null;
        }

        if (acDcMotor != null && acDcLimitSwitch != null) {
            ascenderDescender = new LinearActuator(acDcMotor, acDcLimitSwitch);
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
            public void liveConfigure(DebouncedGamepadButtons buttons) {

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
            public void liveConfigure(DebouncedGamepadButtons buttons) {

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

        operatorsGamepad = new NinjaGamePad(gamepad2);

        // ----------------------
        // Global operator controls
        // ----------------------

        limitOverrideButton = new RangeInputButton(
                operatorsGamepad.getRightTrigger(), 0.65f);

        // MM: Tricky - used multiple places, need each usage to track own state
        OnOffButton eStopButton = operatorsGamepad.getBButton();

        // ----------------------
        // Ascender/Descender controls
        // ----------------------

        // acdc - on-offs

        acDcExtendButton = operatorsGamepad.getDpadUp();

        acDcRetractButton = operatorsGamepad.getDpadDown();

        acDcLimitOverrideButton = limitOverrideButton;

        // acdc - debounced

        acDcStopButton = new DebouncedButton(eStopButton);

        acDcHomeButton = new DebouncedButton(operatorsGamepad.getRightBumper());

        // ----------------------
        // Elevator controls
        // ----------------------

        elevatorCommandUpButton = operatorsGamepad.getLeftBumper();

        elevatorCommandDownButton = new RangeInputButton(
                operatorsGamepad.getLeftTrigger(), .65f);

        elevatorUpperLimitButton = new DebouncedButton(operatorsGamepad.getYButton());

        elevatorLowerLimitButton = new DebouncedButton(operatorsGamepad.getAButton());

        elevatorEmergencyStopButton = new DebouncedButton(eStopButton);

        boxTipButton = operatorsGamepad.getXButton();

        // ----------------------
        // Collector controls
        // ----------------------

        collectorDeployInput = operatorsGamepad.getRightStickY();

        collectorSweepInput = operatorsGamepad.getLeftStickY();

        loadToBoxButton = operatorsGamepad.getDpadRight();
    }

    private final float lowPassFilterFactor = .92F;

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

    protected void handleAscender() {
        if (acDcExtendButton.isPressed()){
            ascenderDescender.extend(acDcLimitOverrideButton.isPressed());
        } else if (acDcRetractButton.isPressed()){
            ascenderDescender.retract(acDcLimitOverrideButton.isPressed());
        } else if (acDcHomeButton.getRise()){
            ascenderDescender.home();
        } else if (acDcStopButton.getRise()) {
            ascenderDescender.stopMoving();
        } else if (!ascenderDescender.isHoming()) {
            ascenderDescender.stopMoving();
        }

        // do the periodic task on the linear actuator
        ascenderDescender.doPeriodicTask();

    }
}
