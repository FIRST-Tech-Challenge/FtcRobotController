package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.apache.commons.math3.stat.regression.SimpleRegression
import kotlin.math.sqrt

abstract class FeedforwardTuningOpMode @JvmOverloads constructor(
        private val distance: Double,
        private val wheelMotorRpm: Double,
        private val wheelDiameter: Double,
        private val wheelGearRatio: Double = 1.0
) : LinearOpMode() {
    companion object {
        // TODO: is padding the derivative acceptable?
        private fun numericalDerivative(x: List<Double>, y: List<Double>): List<Double> {
            val deriv = (0 until x.size - 2).map { (y[it+2] - y[it]) / (x[it+2] - x[it]) }.toMutableList()
            deriv.add(0, deriv[0])
            deriv.add(deriv.last())
            return deriv
        }
    }

    override fun runOpMode() {
        val drive = initDrive()

        telemetry.log().add("Press play to begin the feedforward tuning routine")
        telemetry.update()

        waitForStart()

        telemetry.log().clear()
        telemetry.log().add("Would you like to fit kStatic?")
        telemetry.log().add("Press (A) for yes, (B) for no")
        telemetry.update()

        var fitIntercept = false
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitIntercept = true
                break
            } else if (gamepad1.b) {
                break
            }
            idle()
        }

        telemetry.log().clear()
        telemetry.log().add(String.format("Place your robot on the field with at least %.2f in of room in front"))
        telemetry.log().add("Press (A) to begin")
        telemetry.update()

        while (opModeIsActive() && !gamepad1.a) {
            idle()
        }

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        val maxVel = wheelMotorRpm * wheelGearRatio * Math.PI * wheelDiameter / 60.0
        val finalVel = 0.7 * maxVel
        val accel = (finalVel * finalVel) / (2.0 * distance)
        val t = sqrt(2.0 * distance / accel)

        val startTime = System.nanoTime() / 1e9
        val timeSamples = mutableListOf<Double>()
        val powerSamples = mutableListOf<Double>()
        val positionSamples = mutableListOf<Double>()

        drive.resetPoseEstimate(Pose2d())
        while (true) {
            val elapsedTime = System.nanoTime() / 1e9 - startTime
            if (elapsedTime > t) {
                drive.setVelocity(Pose2d(0.0, 0.0, 0.0))
                break
            }
            val vel = accel * elapsedTime
            val power = vel / maxVel

            timeSamples.add(elapsedTime)
            powerSamples.add(power)
            positionSamples.add(drive.getPoseEstimate().x)

            drive.setVelocity(Pose2d(power, 0.0, 0.0))
            drive.updatePoseEstimate()
        }

        val velocitySamples = numericalDerivative(timeSamples, positionSamples)
        val regression = SimpleRegression(fitIntercept)
        velocitySamples.zip(powerSamples).forEach { regression.addData(it.first, it.second) }
        val kV = regression.slope
        val kStatic = regression.slope

        telemetry.log().clear()
        telemetry.log().add("Quasi-static ramp up test complete")
        if (fitIntercept) {
            telemetry.log().add(String.format("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", kV, kStatic, regression.rSquare))
        } else {
            telemetry.log().add(String.format("kV = %.5f (R^2 = %.2f)", kV, regression.rSquare))
        }
        telemetry.log().add("Place the robot back in its starting position")
        telemetry.log().add("Press (A) to continue")
        telemetry.update()

        // TODO: add kA tuning routine

        while (opModeIsActive()) {
            idle()
        }
    }

    abstract fun initDrive(): Drive
}