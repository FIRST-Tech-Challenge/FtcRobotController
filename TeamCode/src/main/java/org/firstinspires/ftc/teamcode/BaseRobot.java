package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.systems.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

import java.util.HashMap;
import java.util.Map;

/** @noinspection FieldCanBeLocal, unused, RedundantSuppression */
public class BaseRobot {
    public final Map<String, DcMotor> motors = new HashMap<>();
    public final Map<String, Servo> servos = new HashMap<>();
    public final Map<String, Object> sensors = new HashMap<>();
    public final ElapsedTime runtime = new ElapsedTime();
    public final DcMotor frontLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor rearLeftMotor;
    public final DcMotor rearRightMotor;
    public final DynamicInput input;
    public final HardwareMap hardwareMap;
    public final OpMode parentOp;

    public final Telemetry telemetry;
    public final Logger logger;
    public Arm arm;
    public Odometry odometry;

    /**
     * Core robot class that manages hardware initialization and basic
     * functionality.
     * Serves as the foundation for both autonomous and teleop operations.
     * Key Features:
     * - Hardware mapping and initialization
     * - Motor configuration and control
     * - Mechanism management (arm, odometry, etc.)
     * - Drive system implementation
     */
    public BaseRobot(HardwareMap hardwareMap, DynamicInput input, LinearOpMode parentOp, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.parentOp = parentOp;
        this.input = input;
        this.telemetry = telemetry;
        this.logger = new Logger(this);
        // Initialize and configure the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRight");

        // IF A WHEEL IS GOING THE WRONG DIRECTION CHECK WIRING red/black
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors.put("frontLeft", frontLeftMotor);
        motors.put("frontRight", frontRightMotor);
        motors.put("rearLeft", rearLeftMotor);
        motors.put("rearRight", rearRightMotor);

        if (Settings.Deploy.ARM) {
            arm = new Arm(this);
        }

        if (Settings.Deploy.ODOMETRY) {
            odometry = new Odometry(this);
        }
    }

    public void shutDown() {
        logger.stop();
    }

    public void driveGamepads() {
        gamepadPrimary();
        gamepadAuxiliary();
    }

    /**
     * Implements mecanum drive calculations and motor control
     * 
     * @param drivePower  Forward/backward power (-1.0 to 1.0)
     * @param strafePower Left/right strafe power (-1.0 to 1.0)
     * @param rotation    Rotational power (-1.0 to 1.0)
     */
    public void mecanumDrive(double drivePower, double strafePower, double rotation) {
        // Adjust the values for strafing and rotation
        strafePower *= Settings.Movement.strafe_power_coefficient;

        double frontLeft = drivePower + strafePower + rotation;
        double frontRight = drivePower - strafePower - rotation;
        double rearLeft = drivePower - strafePower + rotation;
        double rearRight = drivePower + strafePower - rotation;

        // Normalize the power values to stay within the range [-1, 1]
        double max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(rearLeft), Math.abs(rearRight)));
        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            rearLeft /= max;
            rearRight /= max;
        }

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        rearLeftMotor.setPower(rearLeft);
        rearRightMotor.setPower(rearRight);
    }

    /**
     * Processes primary gamepad inputs for robot movement
     * Handles boost/brake modifiers and applies power to drive system
     */
    public void gamepadPrimary() {
        DynamicInput.Movements directions = input.getMovements();
        DynamicInput.Actions actions = input.getActions();

        double boost = actions.boostAmount;
        double brake = actions.brakeAmount;

        double powerMultiplier = 1 + boost - brake;
        double rotation = directions.rotation;
        double strafePower = directions.x * powerMultiplier;
        double drivePower = directions.y * powerMultiplier;

        /*
         * Drives the motors based on the given power/rotation
         */
        mecanumDrive(drivePower, strafePower, rotation);
    }

    /**
     * Processes auxiliary gamepad inputs for mechanism control
     * Manages arm, claw, and wrist operations
     */
    public void gamepadAuxiliary() {
        if (Settings.Deploy.ARM) {
            DynamicInput.ContextualActions actions = input.getContextualActions();

            if (actions.justRetractActuator) {
                arm.extensor.retract();
            } else if (actions.justExtendActuator) {
                arm.extensor.extend();
            } else if (actions.justGroundActuator) {
                arm.extensor.ground();
            }

            if (actions.clawRight) {
                arm.claw.setRightServo(!arm.claw.openedR);
            }
            if (actions.clawLeft) {
                arm.claw.setLeftServo(!arm.claw.openedL);
            }

            if (actions.wristUp) {
                arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            } else if (actions.wristDown) {
                arm.wrist.setPosition(Wrist.Position.RUNG);
            }
        }
    }

    private void setMode(DcMotor.RunMode mode) {
        // Set motor mode for all motors
        for (DcMotor motor : motors.values()) {
            motor.setMode(mode);
        }
    }

    private boolean areMotorsBusy() {
        // Check if any motor is busy
        for (DcMotor motor : motors.values()) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    private void stopMotors() {
        // Stop all motors
        for (DcMotor motor : motors.values()) {
            motor.setPower(0);
        }
    }
}