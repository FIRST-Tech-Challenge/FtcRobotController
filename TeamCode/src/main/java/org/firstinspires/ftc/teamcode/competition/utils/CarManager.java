package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.other.utils.TwoWDDrivetrain;

import java.util.ArrayList;
import java.util.List;

public class CarManager extends GamepadExtended {

    private final Car CAR;
    private final Motor RIGHT, LEFT, SPINNER, LIFT;

    public CarManager(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, telemetry);
        RIGHT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        LEFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        CAR = new Car(telemetry, RIGHT, LEFT);
        SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void main() {
        CAR.driveWithEncoder((int) Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -100, 100), (int) Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -100, 100));
        if((gamepad2.left_stick_y >= 0.25 || gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
            SPINNER.driveWithEncoder((int) Range.clip(-gamepad2.left_stick_y * 100, -100, 100));
        }else if(gamepad1.right_trigger >= 0.25 || gamepad1.left_stick_y <= -0.25) {
            SPINNER.driveWithEncoder((int) Range.clip(-gamepad2.left_stick_y * 100, -100, 100));
        }else{
            SPINNER.driveWithEncoder(0);
        }
        if((gamepad2.left_stick_x >= 0.25 || gamepad2.left_stick_x <= -0.25) && priority.f3(false)) {
            LIFT.driveWithEncoder((int) Range.clip(-gamepad2.left_stick_x * 100, -100, 100));
        }else if(gamepad1.right_stick_y >= 0.25 || gamepad1.right_stick_y <= -0.25) {
            LIFT.driveWithEncoder((int) Range.clip(-gamepad1.left_stick_x * 100, -100, 100));
        }else{
            LIFT.driveWithEncoder(0);
        }
    }

    @Override
    public void stop() {
        CAR.stop();
        SPINNER.stop();
        LIFT.stop();
    }

    public Car getCar() {
        return CAR;
    }

    public Motor getRight() {
        return RIGHT;
    }

    public Motor getLeft() {
        return LEFT;
    }

    public Motor getSpinner() {
        return SPINNER;
    }

    public Motor getLift() {
        return LIFT;
    }
}
