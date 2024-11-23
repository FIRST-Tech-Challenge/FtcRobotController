package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

import java.lang.Math;

public class IntakeSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;

    private DcMotorEx leftArmJointMotor;
    private DcMotorEx rightArmJointMotor;

    private DcMotorEx linearSlideMotor;

    private ServoEx wristServo;
    private CRServo activeIntakeServo;

    private double speedMultiplier = 1.0;

    private double leftArmJointMotorTicks;
    private double rightArmJointMotorTicks;
    private double armJointMaximumAngleRotated;
    private double armJointMinimumAngleRotated;

    private double linearSlideMotorTicks;
    private double linearSlideDistanceTraveled;

    private PIDController armJointController;
    private double armJointTarget = 0;
    private double armJointLastPower;

    private ElapsedTime armJointTimer;
    private double armJointTimeout;
    private PIDSubsystemState armJointState;

    private PIDController linearSlideController;
    /** The state the arm/linear slide is in: (manual, moving-to-target, or at-target) */
    private double linearSlideTarget = 0;
    private double linearSlideLastPower;

    private ElapsedTime linearSlideTimer;
    private double linearSlideTimeout;
    private PIDSubsystemState linearSlideState;

    private double floorScanningCursor;

    private IntakeConstants.IntakeSubsystemState currentState;

    private boolean isFloorScanningMode;

    private OpMode opMode;

    public IntakeSubsystem(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode; //a little fix so that the subsystem itself can add things into telemetry

        // intake arms motor creation
        leftArmJointMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LEFT_ARM_MOTOR_NAME);
        rightArmJointMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.RIGHT_ARM_MOTOR_NAME);

        // intake arms direction
        leftArmJointMotor.setDirection(Constants.IntakeConstants.LEFT_ARM_MOTOR_DIRECTION);
        rightArmJointMotor.setDirection(Constants.IntakeConstants.RIGHT_ARM_MOTOR_DIRECTION);

        // brake checker if nothing is pressed
        leftArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);
        linearSlideMotor.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideController = new PIDController(Constants.IntakeConstants.LINEAR_SLIDE_P, Constants.IntakeConstants.LINEAR_SLIDE_I, Constants.IntakeConstants.LINEAR_SLIDE_D);
        linearSlideState = PIDSubsystemState.MANUAL;

        // wrist servo creation
        this.wristServo = new SimpleServo(hardwareMap, Constants.IntakeConstants.INTAKE_WRIST_SERVO_NAME, Constants.IntakeConstants.INTAKE_WRIST_SERVO_MIN_ANGLE, Constants.IntakeConstants.INTAKE_WRIST_SERVO_MAX_ANGLE);
        this.activeIntakeServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);

        armJointController = new PIDController(Constants.IntakeConstants.ARM_JOINT_P, Constants.IntakeConstants.ARM_JOINT_I, Constants.IntakeConstants.ARM_JOINT_D);

        floorScanningCursor = 0;
        isFloorScanningMode = false;

        resetLinearSlideEncoders();
        resetArmJointEncoders();
    }

    public void runArmJointWithPID(double targetPosition, double timeout) {
        //set target and linearSlideTimeout for pid system
        double initialDegreeRotated = getArmJointDegreeRotated();
        armJointState = PIDSubsystemState.MOVING_TO_TARGET;
        // If we aren't at the target
        if(armJointTarget > initialDegreeRotated){
            if (this.armJointTarget <= armJointMinimumAngleRotated) {
                armJointState = PIDSubsystemState.AT_TARGET;
                return;
            }
        } else {
            if(this.armJointTarget >= armJointMaximumAngleRotated) {
                armJointState = PIDSubsystemState.AT_TARGET;
                return;
            }
        }
                // Calculate how much we need to move the motor by
        armJointController.setPID(IntakeConstants.ARM_JOINT_P, IntakeConstants.ARM_JOINT_I, IntakeConstants.ARM_JOINT_D);
        double armJointPosition = getArmJointDegreeRotated();
        double power = linearSlideController.calculate(armJointPosition, this.armJointTarget);
        armJointLastPower = power;
        leftArmJointMotor.setPower(power);
        rightArmJointMotor.setPower(power);
        // If the power we are setting is basically none, we are close enough to the target
        if(Math.abs(power) <= Constants.IntakeConstants.ARM_JOINT_PID_POWER_TOLERANCE || isArmJointTimeoutPassed(armJointTimeout, armJointTimer)) {
            armJointState = PIDSubsystemState.AT_TARGET;
            if(armJointMaximumAngleRotated > IntakeConstants.ARM_JOINT_MAXIMUM_ANGLE_ROTATED || armJointMinimumAngleRotated < IntakeConstants.ARM_JOINT_MINIMUM_ANGLE_ROTATED) {
                armJointState = PIDSubsystemState.AT_TARGET;
                leftArmJointMotor.setPower(0.0);
                rightArmJointMotor.setPower(0.0);
            }
        }
    }

    /**
     * Use the PID controller to calculate how fast we should set the motor to. If the motor is moving slow enough,
     * we are close enough and stop moving further.
     */
    public void runLinearSlideWithPID(double targetPosition, double timeout) {
        //set target and linearSlideTimeout for pid system
        this.linearSlideTarget = targetPosition;
        double initialSlideDistanceTraveled = linearSlideDistanceTraveled;
        linearSlideState = PIDSubsystemState.MOVING_TO_TARGET;
        if (linearSlideTimer == null) linearSlideTimer = new ElapsedTime();
        else linearSlideTimer.reset();
        linearSlideTimeout = timeout;
        // If we aren't at the target
        while (linearSlideState == PIDSubsystemState.MOVING_TO_TARGET) {
            if(linearSlideTarget > initialSlideDistanceTraveled) {
                if(this.linearSlideTarget <= linearSlideDistanceTraveled) {
                    leftArmJointMotor.setPower(0.0);
                    rightArmJointMotor.setPower(0.0);
                    armJointState = PIDSubsystemState.AT_TARGET;
                    return;
                }
            } else {
                if (this.linearSlideTarget >= linearSlideDistanceTraveled) {
                    leftArmJointMotor.setPower(0.0);
                    rightArmJointMotor.setPower(0.0);
                    armJointState = PIDSubsystemState.AT_TARGET;
                    return;
                }
            }

            // Calculate how much we need to move the motor by
            linearSlideController.setPID(IntakeConstants.LINEAR_SLIDE_P, IntakeConstants.LINEAR_SLIDE_I, IntakeConstants.LINEAR_SLIDE_D);
            double linearSlidePosition = getLinearSlideDistanceTraveled();
            double power = linearSlideController.calculate(linearSlidePosition, this.linearSlideTarget);
            linearSlideLastPower = power;
            linearSlideMotor.setPower(power);
            // If the power we are setting is basically none, we are close enough to the target
            if(Math.abs(power) <= Constants.IntakeConstants.LINEAR_SLIDE_PID_POWER_TOLERANCE || isLinearSlideTimeoutPassed(linearSlideTimeout,linearSlideTimer)) {
                    linearSlideState = PIDSubsystemState.AT_TARGET;
                    linearSlideMotor.setPower(0.0);
                }
            if(linearSlideDistanceTraveled > IntakeConstants.MAXIMUM_FORWARD_EXTENSION || linearSlideDistanceTraveled < IntakeConstants.MINIMUM_BACKWARD_EXTENSION) {
                linearSlideMotor.setPower(0.0);
                linearSlideState = PIDSubsystemState.AT_TARGET;
            }
        }
    }

    /**
     * Check if the timeout has passed, if is has, reset the timeout and return true.
     *
     * @return True if the timeout has elapsed, false otherwise
     */
    private boolean isLinearSlideTimeoutPassed(double timeout,ElapsedTime linearSlideTimer) {
        if(timeout > 0 && linearSlideTimer.seconds() >= timeout) {
            linearSlideTimeout = 0;
            return true;
        }
        return false;
    }

    /** @return True if the motor is at the target, false otherwise */
    public boolean isAtTarget() {
        return linearSlideState == PIDSubsystemState.AT_TARGET;
    }

    public double getLinearSlideLastPower() {
        return linearSlideLastPower;
    }

    public double getLinearSlideTarget() {
        return this.linearSlideTarget;
    }

    public void activeIntakeServoIn() {
        this.activeIntakeServo.setPower(1.0);
    }

    public void activeIntakeServoOut() {
        this.activeIntakeServo.setPower(-1.0);
    }

    public void stopActiveIntakeServo() {
        this.activeIntakeServo.setPower(0.0);
    }

    /** Moves the wrist servo into position to retract
    */
    public void servoUpPosition() {
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_UP_POSITION);
    }

    /** Moves the wrist servo into position to pick up samples
    */
    public void servoDownPosition() {
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_DOWN_POSITION);
    }

    public void manualFloorScanningMode(double input) {
        if(isFloorScanningMode){
            input = GamepadUtils.deadzone(input, Constants.DriveConstants.DEADZONE);
            this.floorScanningCursor = Range.clip(floorScanningCursor + input, IntakeConstants.FloorScanningMode.FLOOR_SCANNING_CURSOR_MINIMUM_LIMIT , IntakeConstants.FloorScanningMode.FLOOR_SCANNING_CURSOR_MAXIMUM_LIMIT);
            floorScanningMode();
        }
    }

    public void floorScanningMode() {
        double angleOfDepression = Math.toDegrees(Math.atan(Constants.IntakeConstants.FloorScanningMode.ARM_JOINT_AXIS_HEIGHT_FROM_FLOOR / floorScanningCursor));
        wristServo.turnToAngle(angleOfDepression);
        runArmJointWithPID(-angleOfDepression,3);
        runLinearSlideWithPID(IntakeConstants.FloorScanningMode.ARM_JOINT_AXIS_HEIGHT_FROM_FLOOR / Math.sin(Math.toRadians(angleOfDepression)), 5);
    }

    public void changeState(Constants.IntakeConstants.IntakeSubsystemState subsystemState) {
        currentState = subsystemState;
        switch (currentState){
            case FLOOR_SCANNING_MODE:
                floorScanningCursor = 0.0;
                isFloorScanningMode = true;
                break;

            case HIGH_BASKET_POSITION:
                wristServo.turnToAngle(IntakeConstants.intakeControl.HIGH_BASKET_POSITION_WRIST_ANGLE);
                runLinearSlideWithPID(IntakeConstants.intakeControl.HIGH_BASKET_POSITION_LINEAR_SLIDE_LENGTH, IntakeConstants.intakeControl.LINEAR_SLIDE_PID_TIMEOUT);
                runArmJointWithPID(IntakeConstants.intakeControl.HIGH_BASKET_POSITION_ARM_JOINT_ANGLE, IntakeConstants.intakeControl.ARM_JOINT_PID_TIMEOUT);
                break;

            case GET_SPECIMEN_POSITION:
                wristServo.turnToAngle(IntakeConstants.intakeControl.GET_SPECIMEN_POSITION_WRIST_ANGLE);
                runLinearSlideWithPID(IntakeConstants.intakeControl.GET_SPECIMEN_POSITION_LINEAR_SLIDE_LENGTH, IntakeConstants.intakeControl.LINEAR_SLIDE_PID_TIMEOUT);
                runArmJointWithPID(IntakeConstants.intakeControl.GET_SPECIMEN_POSITION_ARM_JOINT_ANGLE,IntakeConstants.intakeControl.ARM_JOINT_PID_TIMEOUT);
                break;

            case HANG_SPECIMEN_POSITION:
                wristServo.turnToAngle(IntakeConstants.intakeControl.HANG_SPECIMEN_POSITION_WRIST_ANGLE);
                runLinearSlideWithPID(IntakeConstants.intakeControl.HANG_SPECIMEN_POSITION_LINEAR_SLIDE_LENGTH,IntakeConstants.intakeControl.LINEAR_SLIDE_PID_TIMEOUT);
                runArmJointWithPID(IntakeConstants.intakeControl.HANG_SPECIMEN_POSITION_ARM_JOINT_ANGLE,IntakeConstants.intakeControl.ARM_JOINT_PID_TIMEOUT);
                break;

            case AT_BAY:
                wristServo.turnToAngle(IntakeConstants.intakeControl.AT_BAY_WRIST_ANGLE);
                runLinearSlideWithPID(IntakeConstants.intakeControl.AT_BAY_LINEAR_SLIDE_LENGTH,IntakeConstants.intakeControl.LINEAR_SLIDE_PID_TIMEOUT);
                runArmJointWithPID(IntakeConstants.intakeControl.AT_BAY_ARM_JOINT_ANGLE,IntakeConstants.intakeControl.ARM_JOINT_PID_TIMEOUT);
                break;
        }
    }
}