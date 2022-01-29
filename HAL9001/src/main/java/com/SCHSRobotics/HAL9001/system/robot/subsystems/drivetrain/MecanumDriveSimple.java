package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.SCHSRobotics.HAL9001.system.config.AutonomousConfig;
import com.SCHSRobotics.HAL9001.system.config.ConfigData;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.localizer.HolonomicDriveEncoderLocalizer;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.SCHSRobotics.HAL9001.util.math.FakeNumpy;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.jetbrains.annotations.NotNull;

/**
 * A simple HAL mecanum drive subsystem. Does not include roadrunner.
 * <p>
 * Creation Date: 1/5/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HolonomicDrivetrain
 * @see Drivetrain
 * @see MecanumDrive
 * @see XDriveSimple
 * @see XDrive
 * @since 1.1.1
 */
public class MecanumDriveSimple extends HolonomicDrivetrain {
    //The names of all the drivetrain's control buttons.
    protected static final String
            DRIVE_STICK = "Drive Stick",
            TURN_STICK = "Turn Stick",
            SPEED_TOGGLE = "Speed Toggle",
            TURN_SPEED_TOGGLE = "Turn Speed Toggle",
            TURN_LEFT_BUTTON = "Turn Left Button",
            TURN_RIGHT_BUTTON = "Turn Right Button";
    //The names of the motors.
    protected final String TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT;
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
     * The constructor for MecanumDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     * @param useConfig   Whether or not the drivetrain uses the HAL config system.
     */
    public MecanumDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        super(robot, driveConfig, topLeft, topRight, botLeft, botRight);

        /*
          Localizer defaults to using ONLY drive encoders. THIS IS A BAD IDEA, but is enabled by default in case people don't want to have to use the IMU for some reason.
          To use the normal roadrunner default localizer, use the HolonomicDriveEncoderIMULocalizer.
         */

        this.localizer = new HolonomicDriveEncoderLocalizer(
                this,
                topLeft,
                topRight,
                botLeft,
                botRight
        );

        usesConfig = useConfig;

        TOP_LEFT = topLeft;
        TOP_RIGHT = topRight;
        BOT_LEFT = botLeft;
        BOT_RIGHT = botRight;

        setReverseType(ReverseType.LEFT);

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(DRIVE_STICK, new Button<Vector2D>(1, Button.VectorInputs.right_stick));
        gamepad.addButton(TURN_STICK, new Button<Double>(1, Button.DoubleInputs.left_stick_x));
        gamepad.addButton(SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_LEFT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_RIGHT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
    }

    /**
     * The constructor for MecanumDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     */
    public MecanumDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, driveConfig, topLeft, topRight, botLeft, botRight, true);
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[]{
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
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
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
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
                setMotorDirection(TOP_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.FORWARD);
                break;
            case RIGHT:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.REVERSE);
                break;
            case FRONT:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.FORWARD);
                break;
            case BACK:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.REVERSE);
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
    public void setDriveStick(Button<Vector2D> driveStick) {
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
     * @param topLeftPower  The power of the top left motor.
     * @param topRightPower The power of the top right motor.
     * @param botLeftPower  The power of the bottom left motor.
     * @param botRightPower The power of the bottom right motor.
     */
    public void setPower(double topLeftPower, double topRightPower, double botLeftPower, double botRightPower) {
        double[] powers = new double[]{topLeftPower, topRightPower, botLeftPower, botRightPower};

        double max = FakeNumpy.max(FakeNumpy.abs(powers));
        FakeNumpy.divide(powers, max > 1 ? max : 1);

        setMotorPower(TOP_LEFT, powers[0]);
        setMotorPower(TOP_RIGHT, powers[1]);
        setMotorPower(BOT_LEFT, powers[2]);
        setMotorPower(BOT_RIGHT, powers[3]);
    }

    @Override
    protected void turnPowerInternal(double power) {
        setPower(-power, power, -power, power);
    }

    @Override
    protected void movePowerInternal(Vector2D power) {
        Vector2D transformedPowerVector = power.copy().rotate(-PI / 4);
        setPower(
                transformedPowerVector.getX(),
                transformedPowerVector.getY(),
                transformedPowerVector.getY(),
                transformedPowerVector.getX()
        );
    }

    /**
     * Causes the drivetrain to turn and move at the same time, making a kind of arc.
     *
     * @param velocity  The velocity to move at.
     * @param turnPower The power to turn at.
     */
    public void arcPower(Vector2D velocity, double turnPower) {
        Vector2D transformedPowerVector = modifyPower(velocity).rotate(PI / 4);
        turnPower = modifyTurnPower(turnPower);

        setPower(
                transformedPowerVector.getX() - turnPower,
                transformedPowerVector.getY() + turnPower,
                transformedPowerVector.getY() - turnPower,
                transformedPowerVector.getX() + turnPower
        );
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

            Vector2D inputVelocity = gamepad.getInput(DRIVE_STICK);
            inputVelocity.rotate(
                    driveMode == DriveMode.STANDARD ? -PI / 2 : -(PI / 2 + localizer.getPoseEstimate().getHeading())
            );

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

            arcPower(inputVelocity, turnPower);
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
        LEFT,
        RIGHT,
        FRONT,
        BACK
    }
}