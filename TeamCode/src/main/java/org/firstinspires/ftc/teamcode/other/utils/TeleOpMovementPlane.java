package org.firstinspires.ftc.teamcode.other.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.competition.utils.GamepadExtended;

public class TeleOpMovementPlane extends GamepadExtended {

    private final Tank DRIVETRAIN;

    public TeleOpMovementPlane(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Tank tank) {
        super(gamepad1, gamepad2, telemetry);
        DRIVETRAIN = tank;
    }

    @Override
    public void main() {
        double left = gamepad1.left_stick_x + gamepad1.left_stick_y * 100.0;
        double right = gamepad1.left_stick_x - gamepad1.left_stick_y * 100.0;
        DRIVETRAIN.driveWithEncoder((int) right, (int) left);
//        if((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
//            spinner.setPower(gamepad2.left_stick_y);
//        }else if(gamepad1.right_trigger >= 0.25) {
//            spinner.setPower(gamepad1.left_stick_y);
//        }

    }

    @Override
    public void stop() {

    }

    public Tank getDrivetrain() {
        return DRIVETRAIN;
    }

}
