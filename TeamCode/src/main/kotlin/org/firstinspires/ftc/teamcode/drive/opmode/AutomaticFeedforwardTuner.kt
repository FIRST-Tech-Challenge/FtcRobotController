package org.firstinspires.ftc.teamcode.drive.opmode

import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_RPM
import org.firstinspires.ftc.teamcode.config.DriveConstants.RUN_USING_ENCODER
import org.firstinspires.ftc.teamcode.config.DriveConstants.rpmToVelocity
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.system.Misc
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.util.LoggingUtil
import org.firstinspires.ftc.teamcode.util.RegressionUtil
import kotlin.math.sqrt

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(group = "drive")
class AutomaticFeedforwardTuner : LinearOpMode() {
    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg(
                "Feedforward constants usually don't need to be tuned " +
                        "when using the built-in drive motor velocity PID."
            )
        }

        val telemetry: Telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        val drive = CDMecanumDrive(HardwareManager(CDConfig(), hardwareMap))

        val clock: NanoClock = NanoClock.system()

        telemetry.addLine("Press play to begin the feedforward tuning routine")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        telemetry.clearAll()
        telemetry.addLine("Would you like to fit kStatic?")
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no")
        telemetry.update()

        var fitIntercept = false
        while (!isStopRequested) {
            if (gamepad1.y) {
                fitIntercept = true
                while (!isStopRequested && gamepad1.y) {
                    idle()
                }
                break
            } else if (gamepad1.b) {
                while (!isStopRequested && gamepad1.b) {
                    idle()
                }
                break
            }
            idle()
        }

        telemetry.clearAll()
        telemetry.addLine(
            Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE
            )
        )
        telemetry.addLine("Press (Y/Δ) to begin")
        telemetry.update()

        while (!isStopRequested && !gamepad1.y) {
            idle()
        }
        while (!isStopRequested && gamepad1.y) {
            idle()
        }

        telemetry.clearAll()
        telemetry.addLine("Running...")
        telemetry.update()

        val maxVel: Double = rpmToVelocity(MAX_RPM)
        val finalVel = MAX_POWER * maxVel
        val accel = (finalVel * finalVel) / (2.0 * DISTANCE)
        val rampTime: Double = sqrt(2.0 * DISTANCE / accel)

        val timeSamples: MutableList<Double> = java.util.ArrayList<Double>()
        val positionSamples: MutableList<Double> = java.util.ArrayList<Double>()
        val powerSamples: MutableList<Double> = java.util.ArrayList<Double>()

        drive.poseEstimate = Pose2d()

        var startTime: Double = clock.seconds()
        while (!isStopRequested) {
            val elapsedTime: Double = clock.seconds() - startTime
            if (elapsedTime > rampTime) {
                break
            }
            val vel = accel * elapsedTime
            val power = vel / maxVel

            timeSamples.add(elapsedTime)
            positionSamples.add(drive.poseEstimate.x)
            powerSamples.add(power)

            drive.setDrivePower(Pose2d(power, 0.0, 0.0))
            drive.updatePoseEstimate()
        }
        drive.setDrivePower(Pose2d(0.0, 0.0, 0.0))

        val rampResult: RegressionUtil.RampResult = RegressionUtil.fitRampData(
            timeSamples, positionSamples, powerSamples, fitIntercept,
            LoggingUtil.getLogFile(
                Misc.formatInvariant(
                    "DriveRampRegression-%d.csv", System.currentTimeMillis()
                )
            )
        )

        telemetry.clearAll()
        telemetry.addLine("Quasi-static ramp up test complete")
        if (fitIntercept) {
            telemetry.addLine(
                Misc.formatInvariant(
                    "kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare
                )
            )
        } else {
            telemetry.addLine(
                Misc.formatInvariant(
                    "kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare
                )
            )
        }
        telemetry.addLine("Would you like to fit kA?")
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no")
        telemetry.update()

        var fitAccelFF = false
        while (!isStopRequested) {
            if (gamepad1.y) {
                fitAccelFF = true
                while (!isStopRequested && gamepad1.y) {
                    idle()
                }
                break
            } else if (gamepad1.b) {
                while (!isStopRequested && gamepad1.b) {
                    idle()
                }
                break
            }
            idle()
        }

        if (fitAccelFF) {
            telemetry.clearAll()
            telemetry.addLine("Place the robot back in its starting position")
            telemetry.addLine("Press (Y/Δ) to continue")
            telemetry.update()

            while (!isStopRequested && !gamepad1.y) {
                idle()
            }
            while (!isStopRequested && gamepad1.y) {
                idle()
            }

            telemetry.clearAll()
            telemetry.addLine("Running...")
            telemetry.update()

            val maxPowerTime = DISTANCE / maxVel

            timeSamples.clear()
            positionSamples.clear()
            powerSamples.clear()

            drive.poseEstimate = Pose2d()
            drive.setDrivePower(Pose2d(MAX_POWER, 0.0, 0.0))

            startTime = clock.seconds()
            while (!isStopRequested) {
                val elapsedTime: Double = clock.seconds() - startTime
                if (elapsedTime > maxPowerTime) {
                    break
                }

                timeSamples.add(elapsedTime)
                positionSamples.add(drive.poseEstimate.x)
                powerSamples.add(MAX_POWER)

                drive.updatePoseEstimate()
            }
            drive.setDrivePower(Pose2d(0.0, 0.0, 0.0))

            val accelResult: RegressionUtil.AccelResult = RegressionUtil.fitAccelData(
                timeSamples, positionSamples, powerSamples, rampResult,
                LoggingUtil.getLogFile(
                    Misc.formatInvariant(
                        "DriveAccelRegression-%d.csv", System.currentTimeMillis()
                    )
                )
            )

            telemetry.clearAll()
            telemetry.addLine("Constant power test complete")
            telemetry.addLine(
                Misc.formatInvariant(
                    "kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare
                )
            )
            telemetry.update()
        }

        while (!isStopRequested) {
            idle()
        }
    }

    companion object {
        var MAX_POWER: Double = 0.7
        var DISTANCE: Double = 100.0 // in
    }
}