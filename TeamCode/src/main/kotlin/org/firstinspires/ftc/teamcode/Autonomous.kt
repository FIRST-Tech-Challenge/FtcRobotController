package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.FollowTrajectoryAction
import java.lang.Thread.sleep
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Autonomous(name = "Kotlin Auto (Blue Left)", group = "Auto Sub-Mode")
class AutoBlueLeft : AutoSuper( Alliance.BLUE,    AllianceSide.LEFT,    Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Blue Right)", group = "Auto Sub-Mode")
class AutoBlueRight : AutoSuper(Alliance.BLUE,    AllianceSide.RIGHT,   Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Red Left)", group = "Auto Sub-Mode")
class AutoRedLeft : AutoSuper(  Alliance.RED,     AllianceSide.LEFT,    Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Red Right)", group = "Auto Sub-Mode")
class AutoRedRight : AutoSuper( Alliance.RED,     AllianceSide.RIGHT,   Pose2d(0.0, 0.0, 0.0))

//@Disabled
// TODO: can replace super constructor with open and override
open class AutoSuper(
    private val alliance: Alliance,
    private val side: AllianceSide,
    private val initialPose: Pose2d
) : OpMode() {
    enum class Alliance {
        RED,
        BLUE
    }
    enum class AllianceSide {
        LEFT,
        RIGHT
    }

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
        shared.drive = MecanumDrive(hardwareMap, initialPose)
    }

    override fun loop() {
        shared.update()
    }

    override fun start() {
        // Game Plan:
        // - Place a purple pixel on the spike mark
        // - Do other stuff (we're working on it!)

        // alias using JVM references
        val drive = shared.drive!!
//        val lsd = shared.lsd!!
//        val march = shared.march!!

//        while (march.detections.isEmpty()) sleep(march.aprilTag.perTagAvgPoseSolveTime.toLong())
//        val detectionPose = march.detections[0].ftcPose
//        val a = drive.pose.position + ((drive.pose.heading + detectionPose.yaw) * Vector2d(detectionPose.x, detectionPose.y))
//        val testAction = drive.actionBuilder(drive.pose).splineToConstantHeading(a, 0.0).build()
        val testAction = drive.actionBuilder(drive.pose).splineToConstantHeading(drive.pose.position + Vector2d(10.0, 0.0), 0.0).build()
        val packet = TelemetryPacket()
        var res = testAction.run(packet)
        res = testAction.run(packet)
//        drive.FollowTrajectoryAction(TimeTrajectory())

//        drive.FollowTrajectoryAction()
        // +X = forward, +Y = left
//        drive.setDrivePowers(
//            PoseVelocity2d(
//                Vector2d(
//                    1.0,
//                    0.0
//                ),
//                0.0
//            )
//        )
//        sleep(1000)
//        drive.setDrivePowers(
//            PoseVelocity2d(
//                Vector2d(
//                    0.0,
//                    0.0
//                ),
//                0.0
//            )
//        )
//        sleep(250)
//        drive.setDrivePowers(
//            PoseVelocity2d(
//                Vector2d(
//                    0.0,
//                    0.0
//                ),
//                1.0
//            )
//        )
//        sleep(500)
//        drive.setDrivePowers(
//            PoseVelocity2d(
//                Vector2d(
//                    0.0,
//                    0.0
//                ),
//                0.0
//            )
//        )


//        drive.actionBuilder(drive.pose)

//        sleep(5000)
//        lsd.setHeight(LSD.SlideHeight.TOP.height)
//        march
//        TODO("Not yet implemented")
    }

    override fun stop() {
        BotShared.storedPose = shared.drive?.pose!!
    }

}