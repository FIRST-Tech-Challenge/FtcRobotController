package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.StandardServo;
import org.firstinspires.ftc.teamcode.competition.utils.Tank;

public class TankTeleOpManager extends TeleOpManager {

    private final Tank TANK;
    private final Motor SPINNER, LIFT_ONE, LIFT_TWO, DUCK;
    private final StandardServo SPINNER_SERVO, LIFT_SERVO, LIFT_DROPPER;
    private final DistanceSensor LIFT_SENSOR;
    private final TeleOpHWDevices DEVICES;


    public TankTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2, TeleOpHWDevices devices) {
        super(gamepad1, function1, gamepad2, function2);
        TANK = new Tank(telemetry, new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.REVERSE), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.REVERSE));
        if(devices.isSpinnerMotorAllowed()) {
            SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_INTAKE), DcMotorSimple.Direction.FORWARD);
        }else{
            SPINNER = null;
        }
        if(devices.isLiftMotorsAllowed()) {
            LIFT_ONE = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_LIFT_ONE), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT_ONE = null;
        }
        if(devices.isLiftMotorsAllowed()) {
            LIFT_TWO = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_LIFT_TWO), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT_TWO = null;
        }
        if(devices.isDuckMotorAllowed()) {
            DUCK = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_DUCK_MOTOR), DcMotorSimple.Direction.FORWARD);
        }else{
            DUCK = null;
        }
        if(devices.isSpinnerServosAllowed()) {
            SPINNER_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_LOWER_ONE));
        }else{
            SPINNER_SERVO = null;
        }
        if(devices.isLiftServoAllowed()) {
            LIFT_SERVO = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_HAND_TURNER_SERVO));
        }else{
            LIFT_SERVO = null;
        }
        if(devices.isLiftDropperAllowed()) {
            LIFT_DROPPER = new StandardServo(hardwareMap, hardwareMap.appContext.getString(R.string.HARDWARE_HAND_GRABBER_SERVO));
        }else{
            LIFT_DROPPER = null;
        }
        if(devices.isLiftSensorAllowed()) {
            LIFT_SENSOR = hardwareMap.get(DistanceSensor.class, hardwareMap.appContext.getString(R.string.HARDWARE_DISTANCE_SENSOR));
        }else{
            LIFT_SENSOR = null;
        }
        DEVICES = devices;
    }

    @Override
    public void main() {
        TANK.driveWithEncoder((int) Range.clip(getGamepadWithFunction1().left_stick_y * 100, -100, 100), (int) Range.clip(getGamepadWithFunction1().right_stick_y * 100, -100, 100));
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
        if(DEVICES.isSpinnerServosAllowed()) {
            if(getGamepadWithFunction4().left_bumper && !getGamepadWithFunction4().right_bumper) {
                SPINNER_SERVO.setPosition(0);
            }else if(!getGamepadWithFunction4().left_bumper && getGamepadWithFunction4().right_bumper) {
                SPINNER_SERVO.setPosition(50);
            }
        }
    }

    @Override
    public void stop() {
        TANK.stop();
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
