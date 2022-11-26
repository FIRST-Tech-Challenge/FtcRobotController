package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Signal indicating the commanded kinematic state of a drive.
 * @param vel robot frame velocity
 * @param accel robot frame acceleration
 */
data class DriveSignal @JvmOverloads constructor(val vel: Pose2d = Pose2d(), val accel: Pose2d = Pose2d())
