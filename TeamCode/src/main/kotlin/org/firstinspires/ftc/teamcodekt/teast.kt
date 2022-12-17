package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rouge.blacksmith.Anvil
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

fun main() {
    val drive = SampleMecanumDrive(null)
    val traj = Anvil.formTrajectory(drive, Pose2d()).waitTime(1.0).build<TrajectorySequence>()
}
