package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)
        val myBot: RoadRunnerBotEntity =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                .followTrajectorySequence { drive ->
                    drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
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
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}