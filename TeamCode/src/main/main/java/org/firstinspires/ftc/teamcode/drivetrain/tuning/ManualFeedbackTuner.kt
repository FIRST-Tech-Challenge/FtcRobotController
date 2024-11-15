package org.firstinspires.ftc.teamcode.drivetrain.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive
import org.firstinspires.ftc.teamcode.drivetrain.ThreeDeadWheelLocalizer
import java.lang.RuntimeException

class ManualFeedbackTuner : LinearOpMode() {
    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        val drive: MecanumDrive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks.toInt() == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks.toInt() == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks.toInt() == 1) {
            throw RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
        }

        waitForStart()

        while (opModeIsActive()) {
            runBlocking(
                drive.actionBuilder(Pose2d(0.0, 0.0, 0.0)).lineToX(DISTANCE).lineToX(0.0).build()
            )
        }
    }

    companion object {
        var DISTANCE: kotlin.Double = 64.0
    }
}
