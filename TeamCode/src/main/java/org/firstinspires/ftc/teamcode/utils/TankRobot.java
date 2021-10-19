package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

public class TankRobot extends GamepadExtended {

    private final Tank TANK;

    public TankRobot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, Tank tank) {
        super(gamepad1, gamepad2, telemetry);
        TANK = tank;
    }

    @Override
    public void main() {
        double left = gamepad1.left_stick_x + gamepad1.left_stick_y * 100.0;
        double right = gamepad1.left_stick_x - gamepad1.left_stick_y * 100.0;
        TANK.driveWithEncoder((int) right, (int) left);
//        if((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
//            spinner.setPower(gamepad2.left_stick_y);
//        }else if(gamepad1.right_trigger >= 0.25) {
//            spinner.setPower(gamepad1.left_stick_y);
//        }

    }
}
