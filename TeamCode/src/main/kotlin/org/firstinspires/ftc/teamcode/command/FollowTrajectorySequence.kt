package org.firstinspires.ftc.teamcode.command

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

class FollowTrajectorySequence(
    private val mecanumDrive: CDMecanumDrive,
    private val sequence: TrajectorySequence
) : CommandBase() {
    override fun initialize() {
        mecanumDrive.followTrajectorySequenceAsync(sequence)
    }

    override fun execute() {
        mecanumDrive.update()
    }

    override fun end(interrupted: Boolean) {
        mecanumDrive.setDrivePower(Pose2d(0.0, 0.0, 0.0))
    }

    override fun isFinished(): Boolean {
        return !mecanumDrive.isBusy
    }
}