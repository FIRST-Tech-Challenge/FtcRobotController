package spudnik.team24288.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.00, -70.00, Math.toRadians(90)))
                .lineToY(-35) // (7, -25) 90 deg
                .strafeTo(new Vector2d(50.00, -35.00)) // (50, -35) 90 deg
                .strafeToLinearHeading(new Vector2d(50, -60), Math.toRadians(-90)) // (50, -60) -90 deg
                .turnTo(Math.toRadians(90)) // 90 deg
                .strafeTo(new Vector2d(60, -60.00)) // (60, -60) 90 deg
                .strafeTo(new Vector2d(60, -35.00)) // (60, -35) 90 deg
                .strafeToLinearHeading(new Vector2d(60, -60), Math.toRadians(-90)) // (60, -60) -90 deg
                .strafeTo(new Vector2d(48.00, -52.00)) // (48, -52)
                .strafeTo(new Vector2d(48, -60)) // (48, -80) -90 deg
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(7.00, -35)).build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}