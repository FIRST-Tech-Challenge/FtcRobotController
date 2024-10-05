package org.ftc8580.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.rowlandhall.meepmeep.MeepMeep
import org.rowlandhall.meepmeep.MeepMeep.Background
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder
import org.rowlandhall.meepmeep.roadrunner.DriveShim

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(800)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(
                    Pose2d(
                        0.0,
                        0.0,
                        0.0
                    )
                )
                    .forward(30.0)
                    .turn(Math.toRadians(90.0))
                    .forward(30.0)
                    .turn(Math.toRadians(90.0))
                    .forward(30.0)
                    .turn(Math.toRadians(90.0))
                    .forward(30.0)
                    .turn(Math.toRadians(90.0))
                    .build()
            }


    meepMeep.setBackground(Background.FIELD_INTOTHEDEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}