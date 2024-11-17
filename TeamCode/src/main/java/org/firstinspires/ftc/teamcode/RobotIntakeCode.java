package org.firstinspires.ftc.teamcode.OpMode.TeleOp.IntakeTest.ServoIntake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotIntake {

    //Declare Intake slide states
    public enum IntakeSlideState {
        INTAKE_START,
        INTAKE_LOWER,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        INTAKE_TRANSFER,
        SAMPLE_TRANSFER
    }

    //Declare Gamepad
    public Gamepad gamepad1;

    //private final Gamepad previousGamepad = new Gamepad();
    //private final Gamepad currentGamepad = new Gamepad();

    //Declare robot
    public IntakeRobotHardware robot;
    //Declare slide state
    public IntakeSlideState intakeSlideState;

    //This is the constructor
    public RobotIntake(IntakeRobotHardware robot, Gamepad gamepad1) {
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.intakeSlideState = IntakeSlideState.INTAKE_START;
    }

    //Set up debounce timer
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    //Set up the at position boolean for intake
    private boolean IN_SlideIsAtPosition (double targetPosition) {
        return (Math.abs(robot.IntakeSlideServo.getPosition() - (targetPosition)) < 0.01);
    }

    //Create a method for the resetting the intake slides                  *Oringinally Intake Retracted Position, might need to change to initial position for clarification*
    public void IntakeSlidesInit(double IntakeSlideServoRetractedPosition, double IntakeArmServoExtendedPosition, double IntakeWristServoNeutralPosition, double IntakeGrabServoOpenPosition) {
        robot.IntakeSlideServo.setPosition(IntakeSlideServoRetractedPosition);
        robot.rightIntakeArmServo.setPosition(IntakeArmServoExtendedPosition);
        robot.leftIntakeArmServo.setPosition(IntakeArmServoExtendedPosition);
        robot.IntakeWristServo.setPosition(IntakeWristServoNeutralPosition);
        robot.IntakeGrabServo.setPosition(IntakeGrabServoOpenPosition);
    }

    //Control the intake wrist servo
    private void IntakeGrab(double IntakeServoSteer) {
        //previousGamepad.copy(currentGamepad);
        //currentGamepad.copy(gamepad1);

        //Rising edge detector
        if (gamepad1.dpad_left) {
            robot.IntakeWristServo.setPosition(robot.IntakeWristServo.getPosition() + IntakeServoSteer);
        }
        if (gamepad1.dpad_right) {
            robot.IntakeWristServo.setPosition(robot.IntakeWristServo.getPosition() - IntakeServoSteer);
        }
    }

    //Intake slide control method
    public void IntakeSlideControl(double IntakeSlideServoExtendedPosition, double IntakeSlideServoRetractPosition, double IntakeArmServoExtendPosition, double IntakeArmServoRetractPosition, double IntakeArmServoGrabPosition, double IntakeWristServoNeutralPosition, double IntakeWristServoSteer, double IntakeGrabServoOpenPosition, double IntakeGrabServoClosePosition) {
        switch (intakeSlideState) {
            case INTAKE_START:
                if (gamepad1.dpad_up && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.IntakeSlideServo.setPosition(IntakeSlideServoExtendedPosition);
                    intakeSlideState = IntakeSlideState.INTAKE_LOWER;
                } else {
                    IntakeSlidesInit(IntakeSlideServoRetractPosition, IntakeArmServoExtendPosition, IntakeWristServoNeutralPosition, IntakeGrabServoOpenPosition);
                }
                break;
            case INTAKE_LOWER:
                if (gamepad1.left_bumper && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.rightIntakeArmServo.setPosition(IntakeArmServoGrabPosition);
                    robot.leftIntakeArmServo.setPosition(IntakeArmServoGrabPosition);
                    intakeSlideState = IntakeSlideState.INTAKE_GRAB;
                } else {
                    IntakeGrab(IntakeWristServoSteer);
                }
                break;
            case INTAKE_GRAB:
                if (gamepad1.right_bumper && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    robot.IntakeGrabServo.setPosition(IntakeGrabServoClosePosition);
                    intakeSlideState = IntakeSlideState.INTAKE_RETRACT;
                }
                else if (gamepad1.left_bumper && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.IntakeGrabServo.setPosition(IntakeGrabServoOpenPosition);
                    robot.rightIntakeArmServo.setPosition(IntakeArmServoExtendPosition);
                    robot.leftIntakeArmServo.setPosition(IntakeArmServoExtendPosition);
                    intakeSlideState = IntakeSlideState.INTAKE_LOWER;
                }
                break;
            case INTAKE_RETRACT:

                robot.rightIntakeArmServo.setPosition(IntakeArmServoExtendPosition);
                robot.leftIntakeArmServo.setPosition(IntakeArmServoExtendPosition);
                robot.IntakeWristServo.setPosition(IntakeWristServoNeutralPosition);

                if (gamepad1.dpad_down && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.IntakeSlideServo.setPosition(IntakeSlideServoRetractPosition);
                    intakeSlideState = IntakeSlideState.INTAKE_TRANSFER;
                }
                break;
            case INTAKE_TRANSFER:
                robot.rightIntakeArmServo.setPosition(IntakeArmServoRetractPosition);
                robot.leftIntakeArmServo.setPosition(IntakeArmServoRetractPosition);
                intakeSlideState = IntakeSlideState.SAMPLE_TRANSFER;

                break;
            case SAMPLE_TRANSFER:
                if (gamepad1.right_bumper && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    //deposit servo goes here
                    robot.IntakeGrabServo.setPosition(IntakeGrabServoOpenPosition);
                    robot.rightIntakeArmServo.setPosition(0.7); //Create another variable later
                    robot.leftIntakeArmServo.setPosition(0.7);
                    intakeSlideState = IntakeSlideState.INTAKE_START;
                }
                break;
            default:
                intakeSlideState = IntakeSlideState.INTAKE_START;
                break;
        }
        if (gamepad1.left_trigger > 0.8 && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && intakeSlideState != IntakeSlideState.INTAKE_START) {
            debounceTimer.reset();
            IntakeSlidesInit(IntakeSlideServoRetractPosition, IntakeArmServoExtendPosition, IntakeWristServoNeutralPosition, IntakeGrabServoOpenPosition);
            intakeSlideState = IntakeSlideState.INTAKE_START;
        }
    }
}
