package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp (name = "Robot Deposit", group =  "FTC_24135_Testing")
public class RobotDepositCode {

    private RobotHardware robot = new RobotHardware();

    public enum DepositSlideState {
        START_POSITION,
        LIFT_EXTEND,
        ARM_WRIST_EXTEND,
        LIFT_RETRACT
    }

    public static int depositSlideMotorExtendPosition = 3222;
    public static int depositSlideMotorRetractPosition = 0;
    public double depositMotorPower = 0.5;
    public double leftDepositArmServoExtendedPosition = 0.83;
    public double leftDepositArmServoInitialPosition = 0;
    public double rightDepositArmServoInitialPosition = 0;
    public double rightDepositArmServoExtendedPosition = 0.83;
    public double depositServoOpenPosition = 0.11;
    public double depositServoClosePosition = 0;
    public double depositWristServoInitialPosition = 0;
    public double depositWristServoExtendedPosition = 0.54;

    public Gamepad gamepad1;




    public DepositSlideState currentState = DepositSlideState.START_POSITION;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime retractTime = new ElapsedTime();

    DepositSlideState depositSlideState = DepositSlideState.START_POSITION;

    public static final double DEBOUNCE_THRESHOLD = 0.25;
    public static final double RETRACTTIME_THRESHOLD = 2.0;

    public void init() {

        robot.depositLeftArmServo.setPosition(0);
        robot.depositRightArmServo.setDirection(Servo.Direction.FORWARD);
        robot.depositLeftArmServo.setPosition(0);
        robot.depositRightArmServo.setDirection(Servo.Direction.REVERSE);
        robot.depositWristServo.setPosition(0);
        robot.depositWristServo.setDirection(Servo.Direction.FORWARD);
        robot.depositClawServo.setPosition(0.11);
        robot.depositClawServo.setDirection(Servo.Direction.FORWARD);
        robot.liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.liftMotorLeft.setPower(depositMotorPower);
        robot.liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.liftMotorRight.setTargetPosition(0);
        robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.liftMotorRight.setPower(depositMotorPower);




    }

    private void DepositSlideControl(int DepositSlideMotorExtendPosition, int DepositSlideMotorRetractPosition, double DepositMotorPower, double LeftDepositArmServoExtendedPosition, double LeftDepositArmServoInitialPosition, double RightDepositArmServoExtendedPosition, double RightDepositArmServoInitialPosition, double DepositServoClosePosition, double DepositServoOpenPosition, double DepositWristServoInitialPosition, double DepositWristServoExtendedPosition){
        switch (currentState) {
            case START_POSITION: {
                robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.liftMotorRight.setTargetPosition(0);
                robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotorRight.setPower(DepositMotorPower);
                robot.liftMotorLeft.setTargetPosition(0);
                robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotorLeft.setPower(DepositMotorPower);
                robot.depositLeftArmServo.setPosition(LeftDepositArmServoInitialPosition);
                robot.depositRightArmServo.setPosition(RightDepositArmServoInitialPosition);
                robot.depositWristServo.setPosition(DepositWristServoInitialPosition);
                currentState = DepositSlideState.LIFT_EXTEND;
            }
            break;
            case LIFT_EXTEND:
                if (gamepad1.y && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(depositServoClosePosition);
                    robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.liftMotorLeft.setTargetPosition(DepositSlideMotorExtendPosition);
                    robot.liftMotorRight.setTargetPosition(DepositSlideMotorExtendPosition);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setPower(DepositMotorPower);
                    robot.liftMotorLeft.setPower(DepositMotorPower);
                    currentState = DepositSlideState.ARM_WRIST_EXTEND;

                }
                break;
            case ARM_WRIST_EXTEND:
                if (DE_IsAtPosition(DepositSlideMotorExtendPosition)) {
                    robot.depositLeftArmServo.setPosition(LeftDepositArmServoExtendedPosition);
                    robot.depositRightArmServo.setPosition(RightDepositArmServoExtendedPosition);
                    robot.depositWristServo.setPosition(DepositWristServoExtendedPosition);
                    currentState = DepositSlideState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (SERVO_IsAtPosition(depositServoOpenPosition) && retractTime.seconds() > RETRACTTIME_THRESHOLD) {
                    robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.liftMotorRight.setTargetPosition(DepositSlideMotorRetractPosition);
                    robot.liftMotorLeft.setTargetPosition(DepositSlideMotorRetractPosition);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setPower(DepositMotorPower);
                    robot.liftMotorLeft.setPower(DepositMotorPower);
                    robot.depositLeftArmServo.setPosition(LeftDepositArmServoInitialPosition);
                    robot.depositRightArmServo.setPosition(RightDepositArmServoInitialPosition);
                    robot.depositClawServo.setPosition(DepositServoOpenPosition);
                    robot.depositWristServo.setPosition(DepositWristServoInitialPosition);
                }
                if (DE_IsAtPosition(DepositSlideMotorRetractPosition)) {
                    currentState = DepositSlideState.START_POSITION;
                }
                break;
            default:
                currentState = DepositSlideState.START_POSITION;
                break;
        }
        if (gamepad1.a && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            DepositSlidesInit(DepositSlideMotorRetractPosition, DepositServoClosePosition, LeftDepositArmServoInitialPosition, RightDepositArmServoInitialPosition, DepositWristServoInitialPosition);
            currentState = DepositSlideState.START_POSITION;
        }
        if (gamepad1.right_bumper && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.depositClawServo.setPosition(depositServoOpenPosition);
        }
        if (gamepad1.right_trigger > 0.5) {
            robot.depositClawServo.setPosition(depositServoClosePosition);
        }
    }
    private boolean DE_IsAtPosition(int targetPosition) {
        return (Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) <= 5) &&
                (Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) <= 5);
    }
    private boolean SERVO_IsAtPosition(double targetPosition) {
        return (Math.abs(robot.depositClawServo.getPosition() - targetPosition) <= 0.01);
    }


        private void DepositSlidesInit ( int MotorTargetPosition, double RightdepositArmServoInitialPosition, double LeftDepositArmServoInitialPosition, double DepositWristServoInitialPosition, double DepositServoInitialPosition) {
            robot.liftMotorLeft.setTargetPosition(MotorTargetPosition);
            robot.liftMotorRight.setTargetPosition(MotorTargetPosition);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(0.4);
            robot.liftMotorRight.setPower(0.4);
            robot.depositLeftArmServo.setPosition(LeftDepositArmServoInitialPosition);
            robot.depositRightArmServo.setPosition(RightdepositArmServoInitialPosition);
            robot.depositClawServo.setPosition(DepositServoInitialPosition);
            robot.depositWristServo.setPosition(DepositWristServoInitialPosition);

        }

}
