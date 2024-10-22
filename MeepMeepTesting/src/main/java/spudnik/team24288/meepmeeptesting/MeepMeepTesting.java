package spudnik.team24288.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    final static String SEQUENCE = "RedTop";
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);



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
                                .lineTo(new Vector2d(12, 66))
                                .lineTo(new Vector2d(60, 66))
                                .build()
                        );
                break;
            case "RedBottom":
                myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 25, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36,  -70, Math.toRadians(90.00)))
                                .lineTo(new Vector2d(-36.00, -66))
                                .lineTo(new Vector2d(50.00, -66.00))
                                .build()
                );
                break;

            case "RedTop":
                myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 25, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12,  -70, Math.toRadians(0)))
                                .lineTo(new Vector2d(12, -66))
                                .lineTo(new Vector2d(60, -66))
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