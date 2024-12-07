package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Arm;
import org.firstinspires.ftc.teamcode.lib.Drivetrain;

@TeleOp(name = "Test Teleop")
public class Teleop extends OpMode {
    Arm arm;
    Drivetrain drivetrain;

    @Override
    public void init() {
        this.arm = new Arm(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1;
        double turn = gamepad1.right_stick_x * -1;

        double left = (forward + turn * 1.5) / 2;
        double right = (forward - turn * 1.5) / 2;

        drivetrain.set_power(left, right);
        arm.set_extend_power(gamepad1.right_stick_y);

        if (gamepad1.a) {
            arm.set_lift_position(1910);
            arm.set_lift_power(1);
        }

        if (gamepad1.b) {
            arm.set_lift_position(300);
            arm.set_lift_power(1);
        }

        if (gamepad1.left_bumper) {
            arm.set_rotate_position(0.33);
        }

        if (gamepad1.right_bumper) {
            arm.set_rotate_position(0);
        }

        if (gamepad1.x) {
            arm.set_collect_power(1);
        } else {
            arm.set_collect_power(0);
        }

        if (gamepad1.y) {
            arm.set_collect_power(-1);
        } else {
            arm.set_collect_power(0);
        }

        telemetry.addData("Lift Position", arm.get_lift_position());
        telemetry.addData("Extend Position", arm.get_extend_position());
        telemetry.addData("Arm Rotate", arm.get_rotate_position());
        telemetry.update();
    }
}
