package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Mechanum;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;

public class MechanumTeleOpManager extends TeleOpManager {

    private final Mechanum MECHANUM;
    private final Motor SPINNER, LIFT;
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
        if(devices.isLiftMotorAllowed()) {
            LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
        }else{
            LIFT = null;
        }
        DEVICES = devices;
    }

    @Override
    public void main() {
        // someone could probably do this math better than me. if you can, please do. i spent 3 weeks on this and i still dont understand it. please
        if(getGamepad1Functions().hasF1()) {
            // do the math
            double rx = -getGamepad1().right_stick_x;
            double x = getGamepad1().left_stick_x;
            double y = getGamepad1().left_stick_y;
            double rightTopPower = (y + x) - rx;
            double rightBottomPower = (y - x) - rx;
            double leftTopPower = (-(y - x)) - rx;
            double leftBottomPower = (-(y + x)) - rx;
            // normal drive code
            MECHANUM.driveWithEncoder((int) Range.clip(rightTopPower * 50, -50, 50), (int) Range.clip(rightBottomPower * 50, -50, 50), (int) Range.clip(leftTopPower * 50, -50, 50), (int) Range.clip(leftBottomPower * 50, -50, 50));
        }else if(getGamepad2Functions().hasF1()) {
            // do the math
            double rx = -getGamepad2().right_stick_x;
            double x = getGamepad2().left_stick_x;
            double y = getGamepad2().left_stick_y;
            double rightTopPower = (y + x) - rx;
            double rightBottomPower = (y - x) - rx;
            double leftTopPower = (-(y - x)) - rx;
            double leftBottomPower = (-(y + x)) - rx;
            // normal drive code
            MECHANUM.driveWithEncoder((int) Range.clip(rightTopPower * 50, -50, 50), (int) Range.clip(rightBottomPower * 50, -50, 50), (int) Range.clip(leftTopPower * 50, -50, 50), (int) Range.clip(leftBottomPower * 50, -50, 50));
        }
        if(getGamepad1Functions().hasF2()) {
            if(DEVICES.isSpinnerMotorAllowed()) {
                SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            }
            if(DEVICES.isLiftMotorAllowed()) {
                if(getGamepad1().right_bumper && !getGamepad1().left_bumper) {
                    LIFT.driveWithEncoder(50);
                }else if(!getGamepad1().right_bumper && getGamepad1().left_bumper){
                    LIFT.driveWithEncoder(-50);
                }else{
                    LIFT.driveWithEncoder(0);
                }
            }
        }else if(getGamepad2Functions().hasF2()) {
            if(DEVICES.isSpinnerMotorAllowed()) {
                SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            }
            if(DEVICES.isLiftMotorAllowed()) {
                if(getGamepad1().right_bumper && !getGamepad1().left_bumper) {
                    LIFT.driveWithEncoder(50);
                }else if(!getGamepad1().right_bumper && getGamepad1().left_bumper){
                    LIFT.driveWithEncoder(-50);
                }else{
                    LIFT.driveWithEncoder(0);
                }
            }
        }
    }

    @Override
    public void stop() {
        MECHANUM.stop();
    }

}
