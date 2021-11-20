package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Mechanum;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.StandardServo;

public class MechanumTeleOpManager extends TeleOpManager {

    private final Mechanum MECHANUM_DRIVETRAIN;
    private final Motor INTAKE_MOTOR, LIFT_MOTOR_ONE, LIFT_MOTOR_TWO, DUCK_MOTOR;
    private final StandardServo INTAKE_SERVO_LOWER, INTAKE_SERVO_UPPER, HAND_TURNER_SERVO, HAND_FLIPPER_SERVO, HAND_GRABBER_SERVO;
    private final DistanceSensor HAND_DISTANCE_SENSOR;
    private final TeleOpSubsystems SUBSYSTEMS;

    private boolean intakeIsDown, handIsOpen;
    private int timeout, handTimeout;
    private final LinearOpMode OP_MODE;

    public MechanumTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2, TeleOpSubsystems subsystems, LinearOpMode opMode) {
        super(gamepad1, function1, gamepad2, function2);
        Motor rt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor rb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        Motor lt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor lb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        MECHANUM_DRIVETRAIN = new Mechanum(telemetry, rt, rb, lt, lb);
        OP_MODE = opMode;
        SUBSYSTEMS = subsystems;
        if(SUBSYSTEMS.isIntakeActive()) {
            INTAKE_MOTOR = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_INTAKE), DcMotorSimple.Direction.FORWARD);
            INTAKE_SERVO_LOWER = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_LOWER));
            INTAKE_SERVO_UPPER = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_UPPER));
        }else{
            INTAKE_MOTOR = null;
            INTAKE_SERVO_LOWER = null;
            INTAKE_SERVO_UPPER = null;
        }
        if(SUBSYSTEMS.isLiftActive()) {
            LIFT_MOTOR_ONE = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_LIFT_ONE), DcMotorSimple.Direction.FORWARD);
            LIFT_MOTOR_TWO = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_LIFT_TWO), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT_MOTOR_ONE = null;
            LIFT_MOTOR_TWO = null;
        }
        if(SUBSYSTEMS.isHandActive()) {
            HAND_GRABBER_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_HAND_GRABBER_SERVO));
            HAND_FLIPPER_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_HAND_FLIPPER_SERVO));
            HAND_TURNER_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_HAND_TURNER_SERVO));
            HAND_DISTANCE_SENSOR = hardwareMap.get(DistanceSensor.class, hardwareMap.appContext.getString(R.string.HARDWARE_DISTANCE_SENSOR));
        }else{
            HAND_GRABBER_SERVO = null;
            HAND_FLIPPER_SERVO = null;
            HAND_TURNER_SERVO = null;
            HAND_DISTANCE_SENSOR = null;
        }
        if(SUBSYSTEMS.isDuckActive()) {
            DUCK_MOTOR = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_DUCK_MOTOR), DcMotorSimple.Direction.FORWARD);
        }else{
            DUCK_MOTOR = null;
        }
    }

    @Override
    public void main() {
        // someone could probably do this math better than me. if you can, please do. i spent 3 weeks on this and i still dont understand it. please
        double rx = -getGamepadWithFunction1().right_stick_x;
        double x = getGamepadWithFunction1().left_stick_y;
        double y = getGamepadWithFunction1().left_stick_x;
        double rightTopPower = (y + x) - rx;
        double rightBottomPower = (y - x) - rx;
        double leftTopPower = (-(y - x)) - rx;
        double leftBottomPower = (-(y + x)) - rx;
        MECHANUM_DRIVETRAIN.driveWithEncoder((int) Range.clip(rightTopPower * 80, -80, 80), (int) Range.clip(rightBottomPower * 80, -80, 80), (int) Range.clip(leftTopPower * 80, -80, 80), (int) Range.clip(leftBottomPower * 80, -80, 80));
        if(SUBSYSTEMS.isIntakeActive()) {
            if(intakeIsDown) {
                INTAKE_MOTOR.driveWithEncoder((int) Range.clip((getGamepadWithFunction2().left_trigger - getGamepadWithFunction2().right_trigger) * 9001, -100, 100));
            }
            if(getGamepadWithFunction2().a) {
                if(OP_MODE.getRuntime() <= timeout) {
                    timeout += OP_MODE.getRuntime();
                    intakeIsDown = !intakeIsDown;
                    if(intakeIsDown) {
                        INTAKE_SERVO_UPPER.setPosition(100);
                        INTAKE_SERVO_LOWER.setPosition(100);
                    }else{
                        INTAKE_SERVO_UPPER.setPosition(0);
                        INTAKE_SERVO_LOWER.setPosition(0);
                    }
                }
            }
        }
        if(SUBSYSTEMS.isLiftActive()) {
            if(getGamepadWithFunction3().left_bumper && !getGamepadWithFunction3().right_bumper) {
                LIFT_MOTOR_ONE.driveWithEncoder(20);
                LIFT_MOTOR_TWO.driveWithEncoder(-20);
            }else if(!getGamepadWithFunction3().left_bumper && getGamepadWithFunction3().right_bumper) {
                LIFT_MOTOR_ONE.driveWithEncoder(-20);
                LIFT_MOTOR_TWO.driveWithEncoder(20);
            }else{
                LIFT_MOTOR_ONE.driveWithEncoder(0);
                LIFT_MOTOR_TWO.driveWithEncoder(0);
            }
        }
        if(SUBSYSTEMS.isHandActive()) {
            if(getGamepadWithFunction4().dpad_up) {
                HAND_FLIPPER_SERVO.setPosition(100);
            }
            if(getGamepadWithFunction4().dpad_down) {
                HAND_FLIPPER_SERVO.setPosition(0);
            }
            if(getGamepadWithFunction4().dpad_right) {
                HAND_TURNER_SERVO.setPosition(0);
            }
            if(getGamepadWithFunction4().dpad_left) {
                HAND_TURNER_SERVO.setPosition(100);
            }
            if((int) HAND_DISTANCE_SENSOR.getDistance(DistanceUnit.INCH) <= 1) {
                HAND_GRABBER_SERVO.setPosition(0);
            }
            if(getGamepadWithFunction4().b) {
                if(OP_MODE.getRuntime() <= handTimeout) {
                    handTimeout += OP_MODE.getRuntime();
                    handIsOpen = !handIsOpen;
                    if(handIsOpen) {
                        HAND_GRABBER_SERVO.setPosition(100);
                    }else{
                        HAND_GRABBER_SERVO.setPosition(0);
                    }
                }
            }
        }
        if(SUBSYSTEMS.isDuckActive()) {
            if(getGamepadWithFunction5().dpad_left && !getGamepadWithFunction5().dpad_right) {
                DUCK_MOTOR.driveWithEncoder(-20);
            }else if(!getGamepadWithFunction5().dpad_left && getGamepadWithFunction5().dpad_right) {
                DUCK_MOTOR.driveWithEncoder(20);
            }else{
                DUCK_MOTOR.driveWithEncoder(0);
            }
        }
        // TODO: servo limit sensor and servo routines
//        if(DEVICES.isLiftDropperAllowed()) {
//            if(getGamepadWithFunction5().a && !droppingInProgress) {
//                droppingInProgress = true;
//                LIFT_DROPPER.setPosition(0);
//                LIFT_DROPPER.setPosition(100);
//            }
//        }
    }

    @Override
    public void stop() {
        MECHANUM_DRIVETRAIN.stop();
        try {
            INTAKE_MOTOR.stop();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_MOTOR_ONE.stop();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_MOTOR_TWO.stop();
        } catch(NullPointerException ignored) {}
        try {
            DUCK_MOTOR.stop();
        } catch(NullPointerException ignored) {}
        try {
            INTAKE_SERVO_UPPER.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            INTAKE_SERVO_LOWER.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            HAND_TURNER_SERVO.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            HAND_GRABBER_SERVO.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            HAND_FLIPPER_SERVO.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            HAND_DISTANCE_SENSOR.close();
        } catch(NullPointerException ignored) {}
    }

}
