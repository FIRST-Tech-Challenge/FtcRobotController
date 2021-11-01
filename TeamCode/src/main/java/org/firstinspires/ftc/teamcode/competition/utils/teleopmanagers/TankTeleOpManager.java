package org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.Tank;

public class TankTeleOpManager extends TeleOpManager {

    private final Tank TANK;
    private final Motor SPINNER, LIFT;

    public TankTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2) {
        super(gamepad1, function1, gamepad2, function2);
        TANK = new Tank(telemetry, new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.REVERSE), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.REVERSE));
        SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void main() {
        if(getGamepad1Functions().hasF1()) {
            TANK.driveWithEncoder((int) Range.clip(getGamepad1().left_stick_y * 100, -100, 100), (int) Range.clip(getGamepad1().right_stick_y * 100, -100, 100));
        }else if(getGamepad2Functions().hasF1()) {
            TANK.driveWithEncoder((int) Range.clip(getGamepad2().left_stick_y * 100, -100, 100), (int) Range.clip(getGamepad2().right_stick_y * 100, -100, 100));
        }
        if(getGamepad1Functions().hasF2()) {
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            int liftSpeed1 = getGamepad1().left_bumper ? 50 : 0;
            int liftSpeed2 = getGamepad1().right_bumper ? -50 : 0;
            int liftSpeed = liftSpeed1 - liftSpeed2;
            LIFT.driveWithEncoder(liftSpeed);
        }else if(getGamepad2Functions().hasF2()) {
            int liftSpeed1 = getGamepad2().left_bumper ? 50 : 0;
            int liftSpeed2 = getGamepad2().right_bumper ? -50 : 0;
            int liftSpeed = liftSpeed1 - liftSpeed2;
            LIFT.driveWithEncoder(liftSpeed);
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad2().left_trigger - getGamepad2().right_trigger) * 100, -100, 100));
        }
    }

    @Override
    public void stop() {
        TANK.stop();
        SPINNER.stop();
        LIFT.stop();
    }

}
