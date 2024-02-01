package spudnik.team24288.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    final static String SEQUENCE = "RedBottom";
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        switch (SEQUENCE) {
            case "BlueBottom":
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, 70, Math.toRadians(270)))
                                .lineTo(new Vector2d(-36.00, 10.00))
                                .lineTo(new Vector2d(58.00, 10.00))
                                .turn(Math.toRadians(-90))
                                .build()
                    );
                break;
            case "BlueTop":
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 70, Math.toRadians(270)))
                                .lineTo(new Vector2d(60, 70))
                                .build()
                        );
                break;
            case "RedBottom":
                myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 25, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12.00, -70.00, Math.toRadians(90)))
                                .lineTo(new Vector2d(60, -70.00))
                                .build()
                );
                break;
            default:
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .build()
                        );
                break;
        }



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}