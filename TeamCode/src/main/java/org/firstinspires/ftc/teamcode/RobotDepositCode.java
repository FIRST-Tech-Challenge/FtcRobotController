package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class RobotDepositCode extends OpMode {

    private RobotHardware robot = new RobotHardware();

    public enum DepositSlideState {
        START_POSITION,
        LIFT_EXTEND,
        ARM_WRIST_EXTEND,
        DUMP,
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




    public DepositSlideState currentState = DepositSlideState.START_POSITION;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime dumpTime = new ElapsedTime();
    private ElapsedTime retractTime = new ElapsedTime();

    DepositSlideState depositSlideState = DepositSlideState.START_POSITION;

    public static final double DEBOUNCE_THRESHOLD = 0.25;
    public static final double DUMPTIME_THRESHOLD = 2.5;
    public static final double RETRACTTIME_THRESHOLD = 2.0;

    public void init() {

        deposit_Arm_Left_Servo.setPosition(0);
        deposit_Arm_Left_Servo.setDirection(Servo.Direction.FORWARD);
        deposit_Arm_Left_Servo.setPosition(0);
        deposit
        deposit_Wrist_Servo.setDirection(Servo.Direction.FORWARD);
        deposit_Wrist_Servo.setPosition(0);
        deposit_Claw_Servo.setDirection(Servo.Direction.FORWARD);
        deposit_Claw_Servo.setPosition(0.11);
        robot.liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        vs_Left_Motor.setTargetPosition(0);
        vs_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vs_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vs_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vs_Left_Motor.setPower(depositMotorPower);
        vs_Right_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        vs_Right_Motor.setTargetPosition(0);
        vs_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vs_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vs_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vs_Right_Motor.setPower(depositMotorPower);




    }

    private void DepositSlideControl(int DepositSlideMotorExtendPosition, int DepositSlideMotorRetractPosition, double DepositMotorPower, double LeftDepositArmServoExtendedPosition, double LeftDepositArmServoInitialPosition, double RightDepositArmServoExtendedPosition, double RightDepositArmServoInitialPosition, double DepositServoClosePosition, double DepositServoOpenPosition, double DepositWristServoInitialPosition, double DepositWristServoExtendedPosition){
        switch (currentState) {
            case START_POSITION: {
                robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                vs_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                vs_Right_Motor.setTargetPosition(0);
                vs_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vs_Right_Motor.setPower(DepositMotorPower);
                vs_Left_Motor.setTargetPosition(0);
                vs_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vs_Left_Motor.setPower(DepositMotorPower);
                deposit_Arm_Left_Servo.setPosition(LeftDepositArmServoInitialPosition);
                deposit_Arm_Right_Servo.setPosition(RightDepositArmServoInitialPosition);
                deposit_Wrist_Servo.setPosition(DepositWristServoInitialPosition);
                currentState = DepositSlideState.LIFT_EXTEND;
            }
            break;
            case LIFT_EXTEND:
                if (gamepad1.y && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    deposit_Claw_Servo.setPosition(depositServoClosePosition);
                    vs_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    vs_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    vs_Left_Motor.setTargetPosition(DepositSlideMotorExtendPosition);
                    vs_Right_Motor.setTargetPosition(DepositSlideMotorExtendPosition);
                    vs_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vs_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vs_Right_Motor.setPower(DepositMotorPower);
                    vs_Left_Motor.setPower(DepositMotorPower);
                    currentState = DepositSlideState.ARM_WRIST_EXTEND;

                }
                break;
            case ARM_WRIST_EXTEND:
                if (DE_IsAtPosition(DepositSlideMotorExtendPosition)) {
                    deposit_Arm_Left_Servo.setPosition(LeftDepositArmServoExtendedPosition);
                    deposit_Arm_Right_Servo.setPosition(RightDepositArmServoExtendedPosition);
                    deposit_Wrist_Servo.setPosition(DepositWristServoExtendedPosition);
                    currentState = DepositSlideState.DUMP;
                }
                break;
            case DUMP:
                if (gamepad1.x && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    dumpTime.reset();
                    debounceTimer.reset();
                    while (dumpTime.seconds() < DUMPTIME_THRESHOLD) {
                        telemetry.addData("DumpTime", dumpTime.seconds());
                        telemetry.update();
                    }
                    dumpTime.reset();
                    if (dumpTime.seconds() > DUMPTIME_THRESHOLD);
                    deposit_Claw_Servo.setPosition(DepositServoOpenPosition);
                    currentState = DepositSlideState.LIFT_RETRACT;



                }
                break;
            case LIFT_RETRACT:
                if (retractTime.seconds() > RETRACTTIME_THRESHOLD) {
                    telemetry.addData("RetractTime", retractTime.seconds());
                    telemetry.update();
                    robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    vs_Right_Motor.setTargetPosition(DepositSlideMotorRetractPosition);
                    vs_Left_Motor.setTargetPosition(DepositSlideMotorRetractPosition);
                    vs_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vs_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vs_Right_Motor.setPower(DepositMotorPower);
                    vs_Left_Motor.setPower(DepositMotorPower);
                    deposit_Arm_Left_Servo.setPosition(LeftDepositArmServoInitialPosition);
                    deposit_Arm_Right_Servo.setPosition(RightDepositArmServoInitialPosition);
                    deposit_Claw_Servo.setPosition(DepositServoOpenPosition);
                    deposit_Wrist_Servo.setPosition(DepositWristServoInitialPosition);
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
            .setPosition(depositServoOpenPosition);
        }
        if (gamepad1.right_trigger > 0.5) {
            deposit_Claw_Servo.setPosition(depositServoClosePosition);
        }
    }
    private boolean DE_IsAtPosition(int targetPosition) {
        return (Math.abs(vs_Left_Motor.getCurrentPosition() - targetPosition) <= 5) &&
                (Math.abs(vs_Right_Motor.getCurrentPosition() - targetPosition) <= 5);
    }


    private void DepositSlidesInit(int MotorTargetPosition, double RightdepositArmServoInitialPosition, double LeftDepositArmServoInitialPosition, double DepositWristServoInitialPosition, double DepositServoInitialPosition) {
        vs_Left_Motor.setTargetPosition(MotorTargetPosition);
        vs_Right_Motor.setTargetPosition(MotorTargetPosition);
        vs_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vs_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vs_Left_Motor.setPower(0.2);
        vs_Right_Motor.setPower(0.2);
        deposit_Arm_Left_Servo.setPosition(LeftDepositArmServoInitialPosition);
        deposit_Arm_Right_Servo.setPosition(RightdepositArmServoInitialPosition);
        deposit_Claw_Servo.setPosition(DepositServoInitialPosition);
        deposit_Wrist_Servo.setPosition(DepositWristServoInitialPosition);

    }



}
