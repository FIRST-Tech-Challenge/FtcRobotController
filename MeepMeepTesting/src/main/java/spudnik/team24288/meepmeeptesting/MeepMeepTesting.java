package spudnik.team24288.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    final static String SEQUENCE = "RedTop";
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder red = myBot.getDrive().actionBuilder(new Pose2d(7.00, -70.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(7.00, -25.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(7.00, -45.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(50.00, -35.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(50.00, -60.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(60.00, -60.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.00, -35.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(48.00, -52.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(48.00, -70.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(7.00, -25.00), Math.toRadians(90.00));
        Action redBuilt = red.build();
        myBot.runAction(redBuilt);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}