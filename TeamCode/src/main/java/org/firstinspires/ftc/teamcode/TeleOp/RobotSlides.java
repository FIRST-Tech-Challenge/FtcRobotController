package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotSlides {

    //Declare robot
    private final RobotHardware robot;

    //Declare gamepad
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    //Declare previous and current gamepad
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad1 = new Gamepad();

    private final ElapsedTime debounceTime = new ElapsedTime();
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    private final ElapsedTime sampleTransferTime = new ElapsedTime();
    private static final double SAMPLE_TRANSFER_THRESHOLD = 0.5;
    private static final double ARM_EXTEND_THRESHOLD = 1;

    private final ElapsedTime retractTime = new ElapsedTime();
    private static final double RETRACT_TIME_THRESHOLD = 2;

    private RobotSlideState robotSlideState;

    //Constructor
    public RobotSlides (Gamepad gamepad1, Gamepad gamepad2, RobotHardware robot) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
        this.robotSlideState = RobotSlideState.INTAKE_START;
    }

    public void robotSlideControl (double intakeSlideServoExtendPosition, double intakeSlideServoRetractPosition, double intakeArmServoLowerPosition, double intakeArmServoExtendPosition, double intakeArmServoMiddlePosition, double intakeArmServoRetractPosition, double intakeRotationServoInitialPosition, double intakeRotationServoSteer, double intakeClawServoGrabPosition, double intakeClawServoOpenPosition, int depositSlideMotorExtendPosition, int depositSlideLeftMotorRetractPosition, int depositSlideRightMotorRetractPosition, double depositMotorPower, double depositArmServoExtendPosition, double depositArmServoRetractPosition, double depositWristServoExtendPosition, double depositWristServoRetractedPosition, double depositClawServoClosePosition, double depositClawServoOpenPosition) {
        switch (robotSlideState) {
            case INTAKE_START:
                if (gamepad1.dpad_up && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeSlideServo.setPosition(intakeSlideServoExtendPosition);
                    robotSlideState = RobotSlideState.INTAKE_LOWER;
                }
                else {
                    intakeSlideInit(intakeSlideServoRetractPosition, intakeArmServoExtendPosition, intakeRotationServoInitialPosition, intakeClawServoOpenPosition);
                }
                break;
            case INTAKE_LOWER:
                if (gamepad1.left_bumper && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeLeftArmServo.setPosition(intakeArmServoLowerPosition);
                    robot.intakeRightArmServo.setPosition(intakeArmServoLowerPosition);
                    robotSlideState = RobotSlideState.INTAKE_GRAB;
                }
                else {
                    intakeSteer(intakeRotationServoSteer);
                }
                break;
            case INTAKE_GRAB:
                if (gamepad1.left_bumper && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeClawServo.setPosition(intakeClawServoGrabPosition);
                    robotSlideState = RobotSlideState.INTAKE_RETRACT;
                }
                if (gamepad1.right_bumper && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeClawServo.setPosition(intakeClawServoOpenPosition);
                    robot.intakeLeftArmServo.setPosition(intakeArmServoExtendPosition);
                    robot.intakeRightArmServo.setPosition(intakeArmServoExtendPosition);
                    robotSlideState = RobotSlideState.INTAKE_LOWER;
                }
                break;
            case INTAKE_RETRACT:
                robot.intakeLeftArmServo.setPosition(intakeArmServoExtendPosition);
                robot.intakeRightArmServo.setPosition(intakeArmServoExtendPosition);
                robot.intakeRotationServo.setPosition(intakeRotationServoInitialPosition);

                if (gamepad1.dpad_down && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeSlideServo.setPosition(intakeSlideServoRetractPosition);
                    robotSlideState = RobotSlideState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER:
                if (gamepad1.left_bumper && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.intakeLeftArmServo.setPosition(intakeArmServoRetractPosition);
                    robot.intakeRightArmServo.setPosition(intakeArmServoRetractPosition);
                    robotSlideState = RobotSlideState.DEPOSIT_START;
                }
                break;
            case DEPOSIT_START:
                if (sampleTransferTime.seconds() > SAMPLE_TRANSFER_THRESHOLD) {
                    sampleTransferTime.reset();
                    robot.depositClawServo.setPosition(depositClawServoClosePosition);
                    robot.intakeClawServo.setPosition(intakeClawServoOpenPosition);
                }
                if (sampleTransferTime.seconds() > ARM_EXTEND_THRESHOLD) {
                    sampleTransferTime.reset();
                    robot.intakeLeftArmServo.setPosition(intakeArmServoMiddlePosition);
                    robot.intakeRightArmServo.setPosition(intakeArmServoMiddlePosition);
                    robotSlideState = RobotSlideState.DEPOSIT_EXTEND;
                }
                break;
            case DEPOSIT_EXTEND:
                if (gamepad2.y && debounceTime.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTime.reset();
                    robot.liftMotorLeft.setTargetPosition(depositSlideMotorExtendPosition);
                    robot.liftMotorRight.setTargetPosition(depositSlideMotorExtendPosition);
                    robot.liftMotorLeft.setPower(depositMotorPower);
                    robot.liftMotorRight.setPower(depositMotorPower);
                    robot.liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    robot.liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    robotSlideState = RobotSlideState.DEPOSIT_EXTEND;
                }
                else {
                    depositSlideInit(depositSlideLeftMotorRetractPosition, depositSlideRightMotorRetractPosition, depositMotorPower);
                }
                break;
            case ARM_EXTEND:
                if (depositSlide_AtPosition(depositSlideMotorExtendPosition, depositSlideMotorExtendPosition)) {
                    robot.depositLeftArmServo.setPosition(depositArmServoExtendPosition);
                    robot.depositRightArmServo.setPosition(depositArmServoExtendPosition);
                    robot.depositWristServo.setPosition(depositWristServoExtendPosition);
                    robotSlideState = RobotSlideState.DEPOSIT_RETRACT;
                }
                break;
            case DEPOSIT_RETRACT:
                if (depositServo_AtPosition(depositClawServoOpenPosition) && retractTime.seconds() > RETRACT_TIME_THRESHOLD) {
                    retractTime.reset();
                    robot.depositWristServo.setPosition(depositWristServoRetractedPosition);
                    robot.depositLeftArmServo.setPosition(depositArmServoRetractPosition);
                    robot.depositRightArmServo.setPosition(depositArmServoRetractPosition);
                    robot.liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    robot.liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    robot.liftMotorLeft.setTargetPosition(depositSlideLeftMotorRetractPosition);
                    robot.liftMotorRight.setTargetPosition(depositSlideRightMotorRetractPosition);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(depositMotorPower);
                    robot.liftMotorRight.setPower(depositMotorPower);
                }
                if (depositSlide_AtPosition(depositSlideLeftMotorRetractPosition, depositSlideRightMotorRetractPosition)) {
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robotSlideState = RobotSlideState.DEPOSIT_START;
                }
                break;
            default:
                robotSlideState = RobotSlideState.DEPOSIT_START;
                break;
        }
    }

    //Declare enum for the slides
    private enum RobotSlideState {
        INTAKE_START,
        INTAKE_LOWER,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        SAMPLE_TRANSFER,
        DEPOSIT_START,
        DEPOSIT_EXTEND,
        ARM_EXTEND,
        DEPOSIT_RETRACT
    }

                                                                        /** Mostly Extended, Retracted in init, Extended is only for readability, feel free to change to position only**/
    private void intakeSlideInit (double intakeSlideServoRetractedPosition, double intakeArmServoExtendedPosition, double intakeRotationServoInitialPosition, double intakeClawServoOpenPosition) {
        robot.intakeSlideServo.setPosition(intakeSlideServoRetractedPosition);
        robot.intakeLeftArmServo.setPosition(intakeArmServoExtendedPosition);
        robot.intakeRightArmServo.setPosition(intakeArmServoExtendedPosition);
        robot.intakeRotationServo.setPosition(intakeRotationServoInitialPosition);
        robot.intakeClawServo.setPosition(intakeClawServoOpenPosition);
    }

    private void intakeSteer (double intakeRotationServoSteer) {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        //rising edge detector
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() + intakeRotationServoSteer);
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() - intakeRotationServoSteer);
        }
    }

    private void depositSlideInit (int depositMotorLeftRetractedPosition, int depositMotorRightRetractedPosition, double depositMotorPower) {
        robot.liftMotorLeft.setTargetPosition(depositMotorLeftRetractedPosition);
        robot.liftMotorRight.setTargetPosition(depositMotorRightRetractedPosition);
        robot.liftMotorLeft.setPower(depositMotorPower);
        robot.liftMotorRight.setPower(depositMotorPower);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        robot.liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    private boolean depositSlide_AtPosition (int depositLeftMotorRetractedPosition, int depositRightMotorRetractedPosition) {
        return (Math.abs(robot.liftMotorLeft.getCurrentPosition() - depositLeftMotorRetractedPosition) <= 5) &&
                (Math.abs(robot.liftMotorRight.getCurrentPosition() - depositRightMotorRetractedPosition) <= 5);
    }
    private boolean depositServo_AtPosition (double servoClawPosition) {
        return (Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) <= 0.01);
    }
}
