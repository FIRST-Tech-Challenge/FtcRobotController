package org.ftc8580.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.rowlandhall.meepmeep.MeepMeep
import org.rowlandhall.meepmeep.MeepMeep.Background
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(800)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDriveTrainType(DriveTrainType.MECANUM)
            .setDimensions(14.0, 17.0)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 14.0)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(
                    Pose2d(
                        30.0,
                        63.25,
                        Math.toRadians(-90.0)
                    )
                )
                    .lineToLinearHeading(Pose2d(30.0, 26.75, Math.toRadians(0.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(45.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(42.0, 26.75, Math.toRadians(0.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(45.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(54.0, 26.75, Math.toRadians(0.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(45.0)))
                    .waitSeconds(2.0)
                    .lineToLinearHeading(Pose2d(24.0, 14.0, Math.toRadians(180.0)))
                    .build()
            }


    meepMeep.setBackground(Background.FIELD_INTOTHEDEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}