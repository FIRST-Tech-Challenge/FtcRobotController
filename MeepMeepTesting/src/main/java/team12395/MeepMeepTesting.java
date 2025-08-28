package team12395;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initPose = new Pose2d(new Vector2d(-20, 40), 0);
        Pose2d endPose = new Pose2d(new Vector2d(40, -20), 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        MasterDrive m = new MasterDrive(myBot);
        m.setEnvironment(2, 15/2., 1);

        TrajectoryActionBuilder n = myBot.getDrive().actionBuilder(initPose);
        //.lineToX(58);

        n = n.lineToX(58);

        myBot.runAction(m.driveToMeepMeep(initPose, endPose)
                //n.build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}