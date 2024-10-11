package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.transfer.ExtendOut
import org.firstinspires.ftc.teamcode.command.transfer.RotateUp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase

class StarterAuton : OpModeBase() {
    override fun initialize() {
        initHardware()

        // Start Pose
        val startingPose = Pose2d(
            12.0,
            63.5,
            Math.toRadians(270.0)
        )

        val nextPose = Pose2d(
            12.0,
            58.5,
            Math.toRadians(270.0)
        )

        val sampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(nextPose)
            .build()

        schedule(
            RotateUp(viperArmSubsystem),
            FollowTrajectorySequence(mecanumDrive, sampleTrajectorySequence),
            ExtendOut(viperArmSubsystem)
        )
    }
}