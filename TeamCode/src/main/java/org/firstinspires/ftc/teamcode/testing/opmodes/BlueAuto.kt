package org.firstinspires.ftc.teamcode.testing.opmodes

import asiankoala.testing.Robot
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
class KAuto : KOpMode() {

    private val startPose = Pose(-70.0, -30.0, 0.0.radians)
    private val startPose2 = Pose(-12.0, -70.0, 50.0.radians)
    private val startPose3 = Pose(-3.0, -30.0, 245.0.radians)
    private val robot by lazy { Robot(startPose) }

    private lateinit var mainCommand: Cmd

    val path1 = PathBuilder(startPose.toPose2d())
        .splineTo(Vector2d(-56.0, -60.0), 330.0.radians)
        .splineTo(Vector2d(-12.0, -70.0), 330.0.radians)
        .build()

    val path2 = PathBuilder(startPose2.toPose2d())
        .splineTo(Vector2d(-3.0, -30.0), 65.0.radians)
        .build()

    val path3 = PathBuilder(startPose3.toPose2d())
        .splineTo(Vector2d(-12.0, -68.0), 270.0.radians)
        .build()

    override fun mInit() {

        mainCommand = SequentialGroup(
            WaitUntilCmd { opmodeState == OpModeState.LOOP },
            GVFCmd(
                robot.drive,
                path1,
                0.7,
                1.0 / 25.0,
                4.0,
                0.8,
                2.0
            ),
            GVFCmd(
                robot.drive,
                path2,
                0.7,
                1.0 / 25.0,
                4.0,
                0.8,
                2.0
            ),
            GVFCmd(
                robot.drive,
                path3,
                0.7,
                1.0 / 25.0,
                4.0,
                0.8,
                2.0
            )
        )
        mainCommand.schedule()
    }
}