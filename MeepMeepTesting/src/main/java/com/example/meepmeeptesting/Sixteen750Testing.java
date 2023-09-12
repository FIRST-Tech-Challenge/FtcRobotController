package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Sixteen750Testing {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setDimensions(18, 17)
            //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.5)
            .followTrajectorySequence(drive ->
                drive
                    .trajectorySequenceBuilder(AutoConstantsBlue.Away.START)
                    .addTrajectory(AutoConstantsBlue.Away.START_TO_LEFT_PARK.get())
                    .build()
            );

        meepMeep
            .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
