package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.koawalib.Robot

@Autonomous
class KAuto : KOpMode() {
    private val robot by lazy { Robot(startPose) }
    private val startPose = Pose(-60.0, -10.0, 0.0.radians)
//
//    val path = PathBuilder(startPose.toPose2d())
//        .splineTo(Vector2d(-55.0, -10.0), 0.0.radians)
//        .build()

    override fun mInit() {
        robot.webcam.device.startStreaming()
    }

    override fun mInitLoop() {
        robot.webcam.start()
    }

    override fun mStart() {
        robot.webcam.device.stopStreaming()
    }

    override fun mLoop() {
        robot.webcam.update()
//        if(robot.webcam.tagOfInterest == null ||  robot.webcam.tagOfInterest!!.id == robot.webcam.LEFT){
//            GVFCmd(
//                robot.drive,
//                path,
//                0.7,
//                1.0 / 25.0,
//                4.0,
//                0.8,
//                2.0
//            )
//        }
//        else if (robot.webcam.tagOfInterest!!.id == robot.webcam.MIDDLE){
//
//        }
//        else {
//
//        }
    }
}