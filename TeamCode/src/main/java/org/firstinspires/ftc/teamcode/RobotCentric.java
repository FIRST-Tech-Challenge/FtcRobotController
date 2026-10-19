package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class RobotCentric extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(

                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                -gamepad1.right_stick_y
        );

        follower.update();
        telemetry.addData("gamepad1_left_stick_y", gamepad1.left_stick_x);
        telemetry.addData("gamepad1_left_stick_x", gamepad1.right_stick_x);
        telemetry.addData("gamepad1_right_stick_x", gamepad1.right_stick_y);
        telemetry.update();
    }
}