package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Mechanum;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.StandardServo;

public class MechanumTeleOpManager extends TeleOpManager {

    private final Mechanum MECHANUM;
    private final Motor SPINNER, LIFT_ONE, LIFT_TWO, DUCK;
    private final StandardServo SPINNER_SERVO, LIFT_SERVO, LIFT_DROPPER;
    private final DistanceSensor LIFT_SENSOR;
    private final TeleOpHWDevices DEVICES;

    public MechanumTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2, TeleOpHWDevices devices) {
        super(gamepad1, function1, gamepad2, function2);
        Motor rt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor rb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        Motor lt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor lb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        MECHANUM = new Mechanum(telemetry, rt, rb, lt, lb);
        if(devices.isSpinnerMotorAllowed()) {
            SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        }else{
            SPINNER = null;
        }
        if(devices.isLiftMotorsAllowed()) {
            LIFT_ONE = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT_ONE = null;
        }
        if(devices.isLiftMotorsAllowed()) {
            LIFT_TWO = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT_TWO), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT_TWO = null;
        }
        if(devices.isDuckMotorAllowed()) {
            DUCK = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_DUCK), DcMotorSimple.Direction.FORWARD);
        }else{
            DUCK = null;
        }
        if(devices.isSpinnerServoAllowed()) {
            SPINNER_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER_SERVO));
        }else{
            SPINNER_SERVO = null;
        }
        if(devices.isLiftServoAllowed()) {
            LIFT_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT_SERVO));
        }else{
            LIFT_SERVO = null;
        }
        if(devices.isLiftDropperAllowed()) {
            LIFT_DROPPER = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT_DROPPER_SERVO));
        }else{
            LIFT_DROPPER = null;
        }
        if(devices.isLiftSensorAllowed()) {
            LIFT_SENSOR = hardwareMap.get(DistanceSensor.class, hardwareMap.appContext.getString(R.string.HW_TRAPPER_TRIGGER));
        }else{
            LIFT_SENSOR = null;
        }
        DEVICES = devices;
    }

    @Override
    public void main() {
        // someone could probably do this math better than me. if you can, please do. i spent 3 weeks on this and i still dont understand it. please
        double rx = -getGamepadWithFunction1().right_stick_x;
        double x = getGamepadWithFunction1().left_stick_x;
        double y = getGamepadWithFunction1().left_stick_y;
        double rightTopPower = (y + x) - rx;
        double rightBottomPower = (y - x) - rx;
        double leftTopPower = (-(y - x)) - rx;
        double leftBottomPower = (-(y + x)) - rx;
        MECHANUM.driveWithEncoder((int) Range.clip(rightTopPower * 50, -50, 50), (int) Range.clip(rightBottomPower * 50, -50, 50), (int) Range.clip(leftTopPower * 50, -50, 50), (int) Range.clip(leftBottomPower * 50, -50, 50));
        if(DEVICES.isSpinnerMotorAllowed()) {
            SPINNER.driveWithEncoder((int) Range.clip((getGamepadWithFunction1().left_trigger - getGamepadWithFunction1().right_trigger) * 100, -100, 100));
        }
        if(DEVICES.isLiftMotorsAllowed()) {
            if(getGamepadWithFunction2().dpad_up && !getGamepadWithFunction2().dpad_down) {
                LIFT_ONE.driveWithEncoder(50);
                LIFT_TWO.driveWithEncoder(50);
            }else if(!getGamepadWithFunction2().dpad_down && getGamepadWithFunction2().dpad_up) {
                LIFT_ONE.driveWithEncoder(-50);
                LIFT_TWO.driveWithEncoder(-50);
            }else{
                LIFT_ONE.driveWithEncoder(0);
                LIFT_TWO.driveWithEncoder(0);
            }
        }
        if(DEVICES.isDuckMotorAllowed()) {
            if(getGamepadWithFunction3().dpad_left && !getGamepadWithFunction3().dpad_right) {
                DUCK.driveWithEncoder(-20);
            }else if(!getGamepadWithFunction3().dpad_left && getGamepadWithFunction3().dpad_right) {
                DUCK.driveWithEncoder(20);
            }else{
                DUCK.driveWithEncoder(0);
            }
        }
        if(DEVICES.isSpinnerServoAllowed()) {
            if(getGamepadWithFunction4().left_bumper && !getGamepadWithFunction4().right_bumper) {
                SPINNER_SERVO.setPosition(0);
            }else if(!getGamepadWithFunction4().left_bumper && getGamepadWithFunction4().right_bumper) {
                SPINNER_SERVO.setPosition(50);
            }
        }
        // TODO: add lift distance sensor and grabber, also get a sensor to detect when this has gone to a specific position, remove this code which is just a demo of what we need at the end of the day
        if(DEVICES.isLiftDropperAllowed()) {
            if(getGamepadWithFunction5().a) {
                LIFT_DROPPER.setPosition(100);
            }else if(getGamepadWithFunction5().b) {
                LIFT_DROPPER.setPosition(0);
            }
        }
        if(DEVICES.isLiftServoAllowed()) {
            if(getGamepadWithFunction6().x) {
                LIFT_SERVO.setPosition(0);
            }else if(getGamepadWithFunction6().y) {
                LIFT_SERVO.setPosition(100);
            }
        }
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
        MECHANUM.stop();
        try {
            SPINNER.stop();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_ONE.stop();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_TWO.stop();
        } catch(NullPointerException ignored) {}
        try {
            DUCK.stop();
        } catch(NullPointerException ignored) {}
        try {
            SPINNER_SERVO.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_SERVO.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_DROPPER.getController().close();
        } catch(NullPointerException ignored) {}
        try {
            LIFT_SENSOR.close();
        } catch(NullPointerException ignored) {}
    }

}
