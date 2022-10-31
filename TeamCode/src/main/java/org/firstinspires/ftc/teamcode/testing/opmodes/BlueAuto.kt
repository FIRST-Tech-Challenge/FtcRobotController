package org.firstinspires.ftc.teamcode.testing.opmodes

import org.firstinspires.ftc.teamcode.testing.Robot
import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline

@Autonomous
class BlueAuto : KOpMode() {

    private val pipeline = SleevePipeline()
    private val startPose = Pose(-59.0, -10.0, 0.0.radians)
    private val robot by lazy { Robot(startPose) }

    override fun mInit() {
        robot.webcam.startStreaming()
    }

    override fun mInitLoop() {
        Logger.addTelemetryData("ROTATION: ", pipeline.position)
    }

    override fun mStart() {
        robot.webcam.stopStreaming()
    }

    override fun mLoop() {
        Logger.addTelemetryData("ROTATION: ", pipeline.position)
//        when (pipeline.position) {
//            SleevePipeline.ParkingPosition.LEFT -> TODO()
//            SleevePipeline.ParkingPosition.CENTER -> TODO()
//            SleevePipeline.ParkingPosition.RIGHT -> TODO()
//        }
    }
}