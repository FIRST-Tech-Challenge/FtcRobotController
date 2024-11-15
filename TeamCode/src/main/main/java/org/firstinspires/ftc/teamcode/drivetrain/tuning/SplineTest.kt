package org.firstinspires.ftc.teamcode.drivetrain.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive
import java.lang.Math.PI

class SplineTest : LinearOpMode() {
    @kotlin.Throws(java.lang.InterruptedException::class)
    override fun runOpMode() {
        val beginPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
        val drive: MecanumDrive = MecanumDrive(hardwareMap, beginPose)

        waitForStart()

        runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(Vector2d(30.0, 30.0), PI / 2)
                .splineTo(Vector2d(0.0, 60.0), PI)
                .build()
        )
    }
}
