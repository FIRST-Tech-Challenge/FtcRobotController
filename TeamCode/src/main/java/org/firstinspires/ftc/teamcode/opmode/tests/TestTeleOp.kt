package org.firstinspires.ftc.teamcode.opmode.tests

import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystem.FieldDrawing


@TeleOp(name = "TeleOp Test", group = "Test")
class TestTeleOp: OpMode() {
    private lateinit var follower: Follower

    override fun init() {
        follower = Constants.create(hardwareMap)
        follower.localizer.setPose(Pose(24.0, 47.0, Math.toRadians(-90.0)))
    }

    override fun start() {
    }

    override fun loop() {
        ManualDrive.driveOrHold(
            follower,
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        );
        follower.update()
    }
}