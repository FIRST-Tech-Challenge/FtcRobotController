package org.firstinspires.ftc.teamcode.koawalib.opmodes

import org.firstinspires.ftc.teamcode.koawalib.Robot
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.KScheduler
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlin.math.PI

@Autonomous
class KAuto : KOpMode() {
    private val robot by lazy { Robot(Pose(heading = 90.0.radians)) }

    override fun mInit() {
        KScheduler.scheduleForStart(
            GVFCmd(
                robot.drive,
                PathBuilder(
                    Pose2d(0.0, 0.0))
                    .splineTo(Vector2d(24.0, 24.0), PI / 2)
                    .build(),
                0.5,
                0.5,
                4.0,
                1.0
            )
        )
    }

}