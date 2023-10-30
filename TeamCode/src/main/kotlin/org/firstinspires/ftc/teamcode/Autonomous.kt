package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import java.lang.Thread.sleep

@Autonomous(name = "Kotlin Auto (Blue Left)", group = "Kt")
class AutoBlueLeft : AutoSuper(  AutoSuper.Alliance.BLUE,    AutoSuper.AllianceSide.LEFT,    Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Blue Right)", group = "Kt")
class AutoBlueRight : AutoSuper(AutoSuper.Alliance.BLUE,    AutoSuper.AllianceSide.RIGHT,   Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Red Left)", group = "Kt")
class AutoRedLeft : AutoSuper(  AutoSuper.Alliance.RED,     AutoSuper.AllianceSide.LEFT,    Pose2d(0.0, 0.0, 0.0))
@Autonomous(name = "Kotlin Auto (Red Right)", group = "Kt")
class AutoRedRight : AutoSuper( AutoSuper.Alliance.RED,     AutoSuper.AllianceSide.RIGHT,   Pose2d(0.0, 0.0, 0.0))

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
        val march = shared.march!!
        val testAction = drive.actionBuilder(drive.pose).splineToConstantHeading(Vector2d(10.0, 0.0), 0.0).build()
        val packet = TelemetryPacket()
        testAction.run(packet)
//        sleep(5000)
//        lsd.setHeight(LSD.SlideHeight.TOP.height)
//        march
//        TODO("Not yet implemented")
    }

    override fun stop() {
        BotShared.storedPose = shared.drive?.pose!!
    }

}