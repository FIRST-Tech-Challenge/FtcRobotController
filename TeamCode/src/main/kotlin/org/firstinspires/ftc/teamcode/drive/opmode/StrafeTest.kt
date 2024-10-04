package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
class StrafeTest : LinearOpMode() {
    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        val telemetry: Telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        val drive = CDMecanumDrive(HardwareManager(CDConfig(), hardwareMap))

        val trajectory: Trajectory = drive.trajectoryBuilder(Pose2d())
            .strafeRight(DISTANCE)
            .build()

        waitForStart()

        if (isStopRequested) return

        drive.followTrajectory(trajectory)

        val poseEstimate: Pose2d = drive.poseEstimate
        telemetry.addData("finalX", poseEstimate.x)
        telemetry.addData("finalY", poseEstimate.y)
        telemetry.addData("finalHeading", poseEstimate.heading)
        telemetry.update()

        while (!isStopRequested && opModeIsActive()) {
            // Do nothing
        }
    }

    companion object {
        var DISTANCE: Double = 60.0 // in
    }
}