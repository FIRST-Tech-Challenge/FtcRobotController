package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive


@Autonomous(name = "Addie's RoadRunner Test", group = "KtTest")
class AddieRunner : LinearOpMode() {

    lateinit var drive: MecanumDrive

    override fun runOpMode() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
//        val tab = TrajectoryActionBuilder(
//            TurnActionFactory       { turn: TimeTurn? -> MecanumDrive.TurnAction(turn)              },
//            TrajectoryActionFactory { t: TimeTrajectory? -> MecanumDrive.FollowTrajectoryAction(t)  },
//            Pose2d(0.0, 0.0, 0.0), 1e-6, 0.0,
//            drive.defaultTurnConstraints,
//            drive.defaultVelConstraint, drive.defaultAccelConstraint,
//            0.25, 0.1
//        )

        drive.updatePoseEstimate()

        val action = drive.actionBuilder(drive.pose)
            .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
            .splineTo(Vector2d(60.0, 0.0), Math.PI)
            .build()

        runBlocking(action)
    }
}