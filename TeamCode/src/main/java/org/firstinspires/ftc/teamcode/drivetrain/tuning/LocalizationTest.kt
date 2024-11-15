package org.firstinspires.ftc.teamcode.drivetrain.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drivetrain.Drawing
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive

class LocalizationTest : LinearOpMode() {
    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive: MecanumDrive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            drive.setDrivePowers(
                PoseVelocity2d(
                    linearVel = Vector2d(
                        x = -gamepad1.left_stick_y.toDouble(),
                        y = -gamepad1.left_stick_x.toDouble()
                    ),
                    angVel = -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.updatePoseEstimate()

            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData(
                "heading (deg)",
                java.lang.Math.toDegrees(drive.pose.heading.toDouble())
            )
            telemetry.update()

            val packet = com.acmerobotics.dashboard.telemetry.TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}
