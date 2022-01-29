package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.SCHSRobotics.HAL9001.system.config.AutonomousConfig;
import com.SCHSRobotics.HAL9001.system.config.ConfigData;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.localizer.NonHolonomicDriveEncoderLocalizer;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.SCHSRobotics.HAL9001.util.functional_interfaces.BiFunction;
import com.SCHSRobotics.HAL9001.util.math.FakeNumpy;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.jetbrains.annotations.NotNull;

/**
 * A simple HAL tank drive subsystem. Does not include roadrunner.
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see NonHolonomicDrivetrain
 * @see Drivetrain
 * @see TankDrive
 * @since 1.1.1
 */
public class TankDriveSimple extends NonHolonomicDrivetrain {
    //The names of all the drivetrain's control buttons.
    protected static final String
            DRIVE_STICK = "Drive Stick",
            TURN_STICK = "Turn Stick",
            SPEED_TOGGLE = "Speed Toggle",
            TURN_SPEED_TOGGLE = "Turn Speed Toggle",
            TURN_LEFT_BUTTON = "Turn Left Button",
            TURN_RIGHT_BUTTON = "Turn Right Button";
    //The list of left and right motor names.
    protected final String[] LEFT_MOTORS, RIGHT_MOTORS;
    //The toggle objects used for the turn speed and velocity toggle buttons.
    private final Toggle
            speedToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false),
            turnSpeedToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    //The gamepad containing the drivetrain controls.
    private CustomizableGamepad gamepad;
    //The multipliers that are applied when the turn speed and velocity toggles are active.
    private double
            speedToggleMultiplier = 0.5,
            turnSpeedToggleMultiplier = 0.5;
    //The power to turn at when the turn left or right buttons are pressed.
    private double turnButtonPower = 0.3;

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param leftMotors  The config names for the motors on the left side of the robot.
     * @param rightMotors The config names for the motors on the right side of the robot.
     * @param useConfig   Whether or not the drivetrain uses the HAL config system.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String[] leftMotors, String[] rightMotors, boolean useConfig) {
        super(robot, driveConfig, ((BiFunction<String[], String[], String[]>) (leftMotors1, rightMotors1) -> {
            String[] motors = new String[leftMotors.length + rightMotors.length];
            System.arraycopy(leftMotors, 0, motors, 0, leftMotors.length);
            System.arraycopy(rightMotors, 0, motors, leftMotors.length, rightMotors.length);
            return motors;
        }).apply(leftMotors, rightMotors));

        usesConfig = useConfig;

        localizer = new NonHolonomicDriveEncoderLocalizer(
                this,
                leftMotors,
                rightMotors
        );

        LEFT_MOTORS = leftMotors.clone();
        RIGHT_MOTORS = rightMotors.clone();

        setReverseType(ReverseType.LEFT);

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(DRIVE_STICK, new Button<Vector2D>(1, Button.DoubleInputs.right_stick_y));
        gamepad.addButton(TURN_STICK, new Button<Double>(1, Button.DoubleInputs.right_stick_x));
        gamepad.addButton(SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_LEFT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_RIGHT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
    }

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param leftMotors  The config names for the motors on the left side of the robot.
     * @param rightMotors The config names for the motors on the right side of the robot.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String[] leftMotors, String[] rightMotors) {
        this(robot, driveConfig, leftMotors, rightMotors, false);
    }

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param leftMotor   The config name for the motor on the left of the robot.
     * @param rightMotor  The config name for the motor on the right side of the robot.
     * @param useConfig   Whether or not the drivetrain uses the HAL config system.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String leftMotor, String rightMotor, boolean useConfig) {
        this(robot, driveConfig, new String[]{leftMotor}, new String[]{rightMotor}, useConfig);
    }

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param leftMotor   The config name for the motor on the left of the robot.
     * @param rightMotor  The config name for the motor on the right side of the robot.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String leftMotor, String rightMotor) {
        this(robot, driveConfig, leftMotor, rightMotor, false);
    }

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     * @param useConfig   Whether or not the drivetrain uses the HAL config system.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        this(robot, driveConfig, new String[]{topLeft, botLeft}, new String[]{topRight, botRight}, useConfig);
    }

    /**
     * The constructor for TankDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     */
    public TankDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, driveConfig, topLeft, topRight, botLeft, botRight, false);
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[]{
                new ConfigParam("Drive Mode", HolonomicDrivetrain.DriveMode.STANDARD),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("Drive Mode", HolonomicDrivetrain.DriveMode.STANDARD),
                new ConfigParam(DRIVE_STICK, Button.VectorInputs.right_stick),
                new ConfigParam(TURN_STICK, Button.DoubleInputs.left_stick_x),
                new ConfigParam(TURN_LEFT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_RIGHT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }

    /**
     * Sets the directions of the drivetrain's motors based on the provided reversetype.
     *
     * @param reverseType Which motors on the drivetrain should be reversed.
     */
    public void setReverseType(@NotNull ReverseType reverseType) {
        switch (reverseType) {
            case LEFT:
                for (String motor : LEFT_MOTORS) {
                    setMotorDirection(motor, DcMotor.Direction.REVERSE);
                }
                for (String motor : RIGHT_MOTORS) {
                    setMotorDirection(motor, DcMotor.Direction.FORWARD);
                }
                break;
            case RIGHT:
                for (String motor : LEFT_MOTORS) {
                    setMotorDirection(motor, DcMotor.Direction.FORWARD);
                }
                for (String motor : RIGHT_MOTORS) {
                    setMotorDirection(motor, DcMotor.Direction.REVERSE);
                }
                break;
        }
    }

    /**
     * Sets the multiplier that is applied when the drivetrain velocity toggle is active.
     *
     * @param velocityMultiplier The multiplier that is applied when the drivetrain velocity toggle is active.
     */
    public void setToggleableVelocityMultiplier(double velocityMultiplier) {
        speedToggleMultiplier = abs(velocityMultiplier);
    }

    /**
     * Sets the multiplier that is applied when the turn speed toggle is active.
     *
     * @param turnSpeedMultiplier The multiplier that is applied when the drivetrain turn speed toggle is active.
     */
    public void setToggleableTurnSpeedMultiplier(double turnSpeedMultiplier) {
        turnSpeedToggleMultiplier = abs(turnSpeedMultiplier);
    }

    /**
     * Sets the power the drivetrain will turn at when the turn left or turn right buttons are pressed.
     *
     * @param turnButtonPower The drivetrain will turn at when the turn left or turn right buttons are pressed.
     */
    public void setTurnButtonPower(double turnButtonPower) {
        this.turnButtonPower = turnButtonPower;
    }

    /**
     * Sets the controller button that will be used to control the drivetrain's velocity.
     *
     * @param driveStick The controller button that will be used to control the drivetrain's velocity.
     */
    public void setDriveStick(Button<Double> driveStick) {
        gamepad.addButton(DRIVE_STICK, driveStick);
    }

    /**
     * Sets the controller button that will be used to control the drivetrain's turn speed.
     *
     * @param turnStick The controller button that will be used to control the drivetrain's turn speed.
     */
    public void setTurnStick(Button<Double> turnStick) {
        gamepad.addButton(TURN_STICK, turnStick);
    }

    /**
     * Sets the controller button that will be used to turn the drivetrain's velocity toggle on and off.
     *
     * @param speedToggleButton The controller button that will be used to turn the drivetrain's velocity toggle on and off.
     */
    public void setSpeedToggleButton(Button<Boolean> speedToggleButton) {
        gamepad.addButton(SPEED_TOGGLE, speedToggleButton);
    }

    /**
     * Sets the controller button that will be used to turn the drivetrain's turn speed toggle on and off.
     *
     * @param turnSpeedToggleButton The controller button that will be used to turn the drivetrain's turn speed toggle on and off.
     */
    public void setTurnSpeedToggleButton(Button<Boolean> turnSpeedToggleButton) {
        gamepad.addButton(TURN_SPEED_TOGGLE, turnSpeedToggleButton);
    }

    /**
     * Sets the button that will turn the drivetrain left.
     *
     * @param turnLeftButton The button that will turn the drivetrain left.
     */
    public void setTurnLeftButton(Button<Boolean> turnLeftButton) {
        gamepad.addButton(TURN_LEFT_BUTTON, turnLeftButton);
    }

    /**
     * Sets the button that will turn the drivetrain right.
     *
     * @param turnRightButton The button that will turn the drivetrain right.
     */
    public void setTurnRightButton(Button<Boolean> turnRightButton) {
        gamepad.addButton(TURN_RIGHT_BUTTON, turnRightButton);
    }

    /**
     * Sets the power to all 4 drivetrain motors. If any motor has a power > 1, it rescales it linearly so that all powers are <= 1.
     *
     * @param leftPower  The power of the left motors.
     * @param rightPower The power of the right motors.
     */
    public void setPower(double leftPower, double rightPower) {
        double[] powers = new double[]{leftPower, rightPower};

        double max = FakeNumpy.max(FakeNumpy.abs(powers));
        FakeNumpy.divide(powers, max > 1 ? max : 1);

        for (String motor : LEFT_MOTORS) {
            setMotorPower(motor, powers[0]);
        }
        for (String motor : RIGHT_MOTORS) {
            setMotorPower(motor, powers[1]);
        }
    }

    @Override
    public void turnPowerInternal(double power) {
        setPower(-power, power);
    }

    @Override
    protected void movePowerInternal(double power) {
        setPower(power, power);
    }

    /**
     * Causes the drivetrain to turn and move at the same time, making a kind of arc.
     *
     * @param power     The power to move at.
     * @param turnPower The power to turn at.
     */
    public void arcPower(double power, double turnPower) {
        power = modifyPower(power);
        turnPower = modifyTurnPower(turnPower);

        setPower(power - turnPower, power + turnPower);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig) {
            gamepad = robot.pullControls(this);
            ConfigData configData = robot.pullNonGamepad(this);

            driveMode = configData.getData("Drive Mode", DriveMode.class);

            velocityScaleMethod = configData.getData("Velocity Scaling Method", SpeedScaleMethod.class);
            turnSpeedScaleMethod = configData.getData("Turn Speed Scaling Method", SpeedScaleMethod.class);

            constantSpeedMultiplier = configData.getData("Velocity Multiplier", Double.class);
            speedToggleMultiplier = configData.getData("Speed Toggle Multiplier", Double.class);
            velocityCap = configData.getData("Velocity Cap", Double.class);

            constantTurnSpeedMultiplier = configData.getData("Turn Speed Multiplier", Double.class);
            turnSpeedToggleMultiplier = configData.getData("Turn Speed Toggle Multiplier", Double.class);
            turnSpeedCap = configData.getData("Turn Speed Cap", Double.class);

            turnButtonPower = configData.getData("Turn Button Power", Double.class);
        }
    }

    @Override
    public void handle() {
        localizer.update();

        if (driveMode != DriveMode.DISABLED) {
            speedToggle.updateToggle(gamepad.getInput(SPEED_TOGGLE));
            turnSpeedToggle.updateToggle(gamepad.getInput(TURN_SPEED_TOGGLE));

            currentSpeedMultiplier = speedToggle.getCurrentState() ? speedToggleMultiplier : 1;
            currentTurnSpeedMultiplier = turnSpeedToggle.getCurrentState() ? turnSpeedToggleMultiplier : 1;

            double power = gamepad.getInput(DRIVE_STICK);

            double turnPower = gamepad.getInput(TURN_LEFT_BUTTON) ? turnButtonPower : gamepad.getInput(TURN_RIGHT_BUTTON) ? -turnButtonPower : 0;
            turnPower = turnPower == 0 ? gamepad.getInput(TURN_STICK) : turnPower;
            turnPower = -turnPower;

            if (turnPower == 0) {
                double correction = abs(headingController.update(localizer.getPoseEstimate().getHeading()));
                correction *= signum(headingController.getLastError()); //cannot be combined into previous line, update() has side effects
                turnPower += abs(headingController.getLastError()) < headingAngleToleranceRadians ? 0 : correction;
            } else {
                headingController.setTargetPosition(localizer.getPoseEstimate().getHeading());
            }

            arcPower(power, turnPower);
        }
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    /**
     * Which motors on the drivetrain are reversed.
     */
    public enum ReverseType {
        LEFT, RIGHT
    }
}
