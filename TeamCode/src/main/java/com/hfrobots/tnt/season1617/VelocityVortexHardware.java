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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.control.RangeInputButton;
import com.hfrobots.tnt.corelib.drive.DriveTrain;
import com.hfrobots.tnt.corelib.drive.DualDcMotor;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.Gear;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.Sprocket;
import com.hfrobots.tnt.corelib.drive.TankDrive;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.Wheel;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Iterator;
import java.util.concurrent.TimeUnit;

public abstract class VelocityVortexHardware extends OpMode {

    protected static final double BEACON_PUSHER_OUT_POSITION = 1;
    protected static final double BEACON_PUSHER_IN_POSITION = 0.12;

    protected NinjaGamePad driversGamepad;

    protected NinjaGamePad operatorsGamepad;

    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected DebouncedButton collectorToggleButton;

    protected DebouncedButton collectorReverseToggleButton;

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

    protected DebouncedButton liftUnlockButton;

    protected DebouncedButton liftLockButton;

    protected ExtendedDcMotor collectorMotor;

    protected TankDrive drive;

    protected DcMotor topParticleShooter;

    protected DcMotor bottomParticleShooter;

    protected ModernRoboticsI2cGyro gyro;

    protected OnOffButton shooterTrigger;

    protected ExtendedDcMotor leftMotor1;
    protected ExtendedDcMotor rightMotor1;
    protected ExtendedDcMotor leftMotor2;
    protected ExtendedDcMotor rightMotor2;

    protected OnOffButton liftSafety;

    protected RangeInput liftThrottle;

    protected DcMotor liftMotor;

    protected DriveTrain liftDriveTrain;

    protected OnOffButton brakeNoBrake;

    protected OnOffButton halfSpeed;

    protected OnOffButton directionFlip;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected Servo liftLockServo;

    protected Servo forkTiltServo;

    protected Servo beaconPusherUnderColorSensor;

    protected Servo beaconPusherNoColorSensor;


    protected final double FORK_TILT_SERVO_REST_POSITION = 0.89;

    protected double forkTiltServoPosition = FORK_TILT_SERVO_REST_POSITION;

    protected VoltageSensor voltageSensor;

    protected ModernRoboticsI2cColorSensor beaconColorSensor;

    protected boolean shooterOn = false; // track state to not log every time through loop()

    protected boolean shooterReverse = false;

    protected OnOffButton particleShooterBouncy;

    protected DebouncedButton particleShooterDebounced;

    protected DebouncedButton grabBallButton;

    protected OpticalDistanceSensor inboardLineSensor;

    protected OpticalDistanceSensor outboardLineSensor;

    //protected ModernRoboticsI2cRangeSensor redAllianceWallRangeSensor;

    //protected ModernRoboticsI2cRangeSensor blueAllianceWallRangeSensor;


    /**
     * Initialize the hardware ... this class requires the following hardware map names
     *
     * DcMotor name="topParticleShooter"
     * DcMotor name="bottomParticleShooter"
     *
     * DcMotor name="leftDrive1"
     * DcMotor name="leftDrive2"
     *
     * DcMotor name="rightDrive1"
     * DcMotor name="rightDrive2"
     *
     * DcMotor name="collectorMotor"
     * DcMotor name="liftMotor"
     */
    @Override
    public void init() {

        try {
            setupDriverControls();

            setupOperatorControls();

            setupLiftAndForks();

            setupBeaconPusher();

            setupDrive();

            setupParticleShooter();

            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

            setupLiftDriveTrain();

            Iterator<VoltageSensor> voltageSensors = hardwareMap.voltageSensor.iterator();
            if (voltageSensors.hasNext()) {
                voltageSensor = voltageSensors.next();
            }

            inboardLineSensor = hardwareMap.opticalDistanceSensor.get("inboardLineSensor");
            outboardLineSensor = hardwareMap.opticalDistanceSensor.get("outboardLineSensor");
        } catch (NullPointerException npe) {
            Log.d("VV", "NPE", npe);
            throw npe;
        }
    }

    protected void setupParticleShooter() {
        collectorMotor = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("collectorMotor"));
        topParticleShooter = hardwareMap.dcMotor.get("topParticleShooter");
        bottomParticleShooter = hardwareMap.dcMotor.get("bottomParticleShooter");
        bottomParticleShooter.setDirection(DcMotorSimple.Direction.REVERSE); // rotates opposite top
    }

    protected void setupDrive() {
        leftMotor1 = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("leftDrive1"));
        leftMotor2 = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("leftDrive2"));
        rightMotor1 = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("rightDrive1"));
        rightMotor2 = NinjaMotor.asNeverest40(hardwareMap.dcMotor.get("rightDrive2"));
        DualDcMotor leftMotor = new DualDcMotor(leftMotor1, leftMotor2);
        DualDcMotor rightMotor = new DualDcMotor(rightMotor1, rightMotor2);

        Wheel stealthWheel = Wheel.andyMarkStealth();
        Gear dummyGear = new Gear(1);

        DriveTrain leftDriveTrain = new DriveTrain("leftDrive", stealthWheel, Rotation.CCW, leftMotor, new Gear[]{dummyGear, dummyGear});
        DriveTrain rightDriveTrain = new DriveTrain("rightDrive", stealthWheel, Rotation.CW, rightMotor, new Gear[]{dummyGear, dummyGear});
        drive = new TankDrive(leftDriveTrain, rightDriveTrain);
    }

    protected void setupBeaconPusher() {
        beaconColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "beaconColorSensor");
        beaconPusherNoColorSensor = hardwareMap.servo.get("beaconPusherNoColorSensor");
        beaconPusherUnderColorSensor = hardwareMap.servo.get("beaconPusherUnderColorSensor");
        beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
        beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
        //redAllianceWallRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "redAllianceWallRangeSensor");
        //blueAllianceWallRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "blueAllianceWallRangeSensor");
        //blueAllianceWallRangeSensor.setI2cAddress(I2cAddr.create8bit(0x30));
    }

    protected void setupLiftAndForks() {
        liftLockServo = hardwareMap.servo.get("liftLockServo");
        forkTiltServo = hardwareMap.servo.get("forkTiltServo");
        lockForks();
        tiltForkServoToStartingPosition();
    }

    private void setupLiftDriveTrain() {
        // A drive train to allow us to run some portions of the lift automated
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        Wheel spoolWheel = new Wheel(1);
        ExtendedDcMotor liftNinjaMotor = NinjaMotor.asNeverest40(liftMotor);
        liftNinjaMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftDriveTrain = new DriveTrain("Cap ball lift", spoolWheel,
                Turn.invert(liftNinjaMotor.getMotorNativeDirection()), liftNinjaMotor ,
                new Sprocket[] { new Sprocket(1), new Sprocket(1)});
    }

    protected void tiltForkServoToStartingPosition() {
        forkTiltServo.setPosition(FORK_TILT_SERVO_REST_POSITION);
    }

    private void setupOperatorControls() {
        // Operator controls
        operatorsGamepad = new NinjaGamePad(gamepad2);
        collectorToggleButton = new DebouncedButton(driversGamepad.getBButton() /* operatorsGamepad.getAButton() */);
        liftLockButton = new DebouncedButton(operatorsGamepad.getXButton());
        liftUnlockButton = new DebouncedButton(operatorsGamepad.getYButton());
        collectorReverseToggleButton = new DebouncedButton(operatorsGamepad.getBButton());
        shooterTrigger = operatorsGamepad.getRightBumper();
        liftSafety = new RangeInputButton(operatorsGamepad.getLeftTrigger(), 0.65f);
        liftThrottle = operatorsGamepad.getLeftStickY();
        grabBallButton = new DebouncedButton(new RangeInputButton(operatorsGamepad.getRightTrigger(), 0.65f));

    }

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();

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
        brakeNoBrake = driversGamepad.getRightBumper();
        halfSpeed = new RangeInputButton(driversGamepad.getLeftTrigger(), 0.65f);
        directionFlip = new RangeInputButton(driversGamepad.getRightTrigger(), 0.65f);
        particleShooterBouncy = driversGamepad.getLeftBumper();
        particleShooterDebounced = new DebouncedButton(particleShooterBouncy);
    }

    protected void particleCollectorOff() {
        collectorMotor.setPower(0);
    }

    protected void runParticleCollectorOutwards() {
        collectorMotor.setPower(-1);
    }

    protected void runParticleCollectorInwards() {
        collectorMotor.setPower(1);
    }

    protected void unlockForks() {
        liftLockServo.setPosition(0.09);
    }

    protected void lockForks() {
        liftLockServo.setPosition(0.647);
    }

    protected void tiltForksBack(double amountMore) {
        forkTiltServoPosition -= amountMore;

        if (forkTiltServoPosition <= 0) {
            forkTiltServoPosition = 0;
        }

        Log.d("VV", "fork tilt servo position = " + forkTiltServoPosition);

        forkTiltServo.setPosition(forkTiltServoPosition);
    }

    protected void tiltForksForward(double amountMore) {
        forkTiltServoPosition += amountMore;

        if (forkTiltServoPosition >= FORK_TILT_SERVO_REST_POSITION) {
            forkTiltServoPosition = FORK_TILT_SERVO_REST_POSITION;
        }

        forkTiltServo.setPosition(forkTiltServoPosition);
    }

    protected void logBatteryState(String opModeMethod) {
        if (voltageSensor == null) {
            Log.e("VV", String.format("No voltage sensor when logging voltage for %s"));

            return;
        }

        Log.d("VV", String.format("Robot battery voltage %5.2f at method %s()",voltageSensor.getVoltage(), opModeMethod));
    }

    protected void shooterOff() {
        if (shooterOn) {
            Log.d("VV", "Particle shooter on");
            shooterOn = false;
            shooterReverse = false;
        }
        topParticleShooter.setPower(0);
        bottomParticleShooter.setPower(0);
    }

    protected void shooterOn() {
        if (!shooterOn) {
            Log.d("VV", "Particle shooter on");
            shooterOn = true;
            shooterReverse = false;
        }
        topParticleShooter.setPower(1);
        bottomParticleShooter.setPower(1);
    }

    protected void shooterReverse() {
        if (!shooterReverse) {
            Log.d("VV", "Particle shooter on");
            shooterReverse = true;
        }
        topParticleShooter.setPower(-0.3);
        bottomParticleShooter.setPower(-0.3);
    }

    protected void addShooterStateMachine(StateMachine stateMachine,
                                          State startShooterState,
                                          State stopShooterState, State collectorEndState, boolean forTeleop) {
        stateMachine.addSequential(startShooterState);
        stateMachine.addSequential(new CollectorOffState(telemetry));

        DelayState waitForCollectorOffState = new DelayState("wait for collector stop", telemetry, 500, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(waitForCollectorOffState);

        // Unjam the loader
        stateMachine.addSequential(new ShooterReverseState(telemetry));
        DelayState waitForReverseState = new DelayState("wait for shooter reverse", telemetry, 150, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(waitForReverseState);
        stateMachine.addSequential(new ShooterOffState(telemetry));

        stateMachine.addSequential(new ShooterOnState(telemetry));

        DelayState waitForShooterSpeedState = new DelayState("wait for shooter speed", telemetry, 500, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(waitForShooterSpeedState);


        stateMachine.addSequential(new CollectorOnState(telemetry));

        // Wait for stop shooting signal release
        stateMachine.addSequential(stopShooterState);

        // Done shooting
        stateMachine.addSequential(new ShooterOffState(telemetry));

        // varies depending on when the shooter is being used!
        stateMachine.addSequential(collectorEndState);

        // Reset all delays
        ResetDelaysState resetAllDelaysState = new ResetDelaysState(telemetry,
                waitForCollectorOffState,  waitForShooterSpeedState, waitForReverseState);
        stateMachine.addSequential(resetAllDelaysState);

        if (forTeleop) {
            resetAllDelaysState.setNextState(startShooterState); // back to the beginning!
        }
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
                    // TODO: "Hold" mode
                    drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.drivePower(0, 0);

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

    class WaitForButton extends State {
        private final OnOffButton trigger;

        public WaitForButton(OnOffButton trigger, Telemetry telemetry) {
            super("Wait for button", telemetry);
            this.trigger = trigger;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (trigger.isPressed()) {
                Log.d("VV", "Shooter trigger pressed");
                return nextState;
            } else {
                return this;
            }
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class WaitForButtonRelease extends State {
        private final OnOffButton trigger;

        public WaitForButtonRelease(OnOffButton trigger, Telemetry telemetry) {
            super("Wait for button release", telemetry);
            this.trigger = trigger;
        }

        @Override
        public State doStuffAndGetNextState() {
            if (trigger.isPressed()) {
                return this;
            } else {
                Log.d("VV", "shooter - trigger released");
                return nextState;
            }
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterReverseState extends State {
        public ShooterReverseState(Telemetry telemetry) {
            super("Particle Shooter Reverse", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterReverse();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterOnState extends State {
        public ShooterOnState(Telemetry telemetry) {
            super("Particle Shooter On", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterOn();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ShooterOffState extends State {
        public ShooterOffState(Telemetry telemetry) {
            super("Particle Shooter Off", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            shooterOff();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class CollectorRunningOutwardsState extends State {

        public CollectorRunningOutwardsState(Telemetry telemetry) {
            super("Collector run outwards", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            runParticleCollectorOutwards();
            return nextState;
        }

        @Override
        public void resetToStart() {
            particleCollectorOff();
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class CollectorOnState extends State {
        public CollectorOnState(Telemetry telemetry) {
            super("Collector Shooter On", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            runParticleCollectorInwards();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class CollectorOffState extends State {
        public CollectorOffState(Telemetry telemetry) {
            super("Collector Shooter Off", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            particleCollectorOff();
            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }

    class ResetDelaysState extends State {
        private final DelayState[] delayStates;

        public ResetDelaysState(Telemetry telemetry, DelayState ... delayStates) {
            super("Reset delays", telemetry);
            this.delayStates = delayStates;
        }

        @Override
        public State doStuffAndGetNextState() {
            for (DelayState state : delayStates) {
                state.resetToStart();
            }

            return nextState;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }





}
