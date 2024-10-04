package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 *
 *
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 *
 *
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */
@Config
@Autonomous(group = "drive")
class MaxAngularVeloTuner : LinearOpMode() {
    private var maxAngVelocity = 0.0

    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        val drive = CDMecanumDrive(HardwareManager(CDConfig(), hardwareMap))

        val telemetry: Telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("Your bot will turn at full speed for $RUNTIME seconds.")
        telemetry.addLine("Please ensure you have enough space cleared.")
        telemetry.addLine("")
        telemetry.addLine("Press start when ready.")
        telemetry.update()

        waitForStart()

        telemetry.clearAll()
        telemetry.update()

        drive.setDrivePower(Pose2d(0.0, 0.0, 1.0))
        val timer = ElapsedTime()

        while (!isStopRequested && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate()

            val poseVelo = drive.poseVelocity

            if (poseVelo != null) {
                maxAngVelocity = poseVelo.heading.coerceAtLeast(maxAngVelocity)
            }
        }

        drive.setDrivePower(Pose2d())

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity)
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity))
        telemetry.addData("Max Recommended Angular Velocity (rad)", maxAngVelocity * 0.8)
        telemetry.addData("Max Recommended Angular Velocity (deg)", Math.toDegrees(maxAngVelocity * 0.8))
        telemetry.update()

        while (!isStopRequested) idle()
    }

    companion object {
        var RUNTIME: Double = 4.0
    }
}