package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rouge.blacksmith.Anvil
import ftc.rouge.blacksmith.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

val v = SampleMecanumDrive.getVelocityConstraint(
    DriveConstants.MAX_VEL,
    DriveConstants.MAX_ANG_VEL,
    DriveConstants.TRACK_WIDTH * .97
)!!

val a = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)!!

fun main() {
    val drive = SampleMecanumDrive(null)
    val traj = Anvil.formTrajectory(drive, Pose2d()).waitSeconds(1.0).build<TrajectorySequence>()
}
