package com.bc4h14169.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Sample1 {
    public static void main(String[] args) {

        /*
            1. Start facing the middle red bar code (done)
            2. Move towards the shipping station (done)
            3. Pretend to drop something off (wait 6 secs for now)
            4. Rotate and face the duck station
            5. Move towards the duck station
            6. Pretend to move ducks (wait 6 secs)
            7. Position robot to face true 90
            8. Move to dock (place with shipping elements)
            9. Face shipping elements
            10. pretend to pick up a shipping element (1 sec)
            11. Position robot to face true 90
            12. Move pass barrier
            13 Deliver to shipping hub
            14. Turn back to end dock



         */
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(41.065033847087705, 41.065033847087705, Math.toRadians(180), Math.toRadians(180), 13.2435)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-37, 72, Math.toRadians(270)))
                                    .splineTo(new Vector2d(-15,40),Math.toRadians(270))
                                    .addDisplacementMarker(()->{}) //step 3
                                    .waitSeconds(3) //step 3
                                    .turn(Math.toRadians(-135)) //step 4
                                    .lineTo(new Vector2d(-46,55))//step 5
                                    .addDisplacementMarker(()->{}) //step 6
                                    .waitSeconds(3)
                                    .turn(Math.toRadians(135)) //step 7
                                    .back(10) //step 8
                                    .strafeTo(new Vector2d(52,67)) //step 8
                                    .turn(Math.toRadians(90)) //step 9
                                    .addDisplacementMarker(()->{}) //step 10
                                    .waitSeconds(1) //step 10
                                    .turn(Math.toRadians(-90)) //step 11
                                    .strafeTo(new Vector2d(11,67)) //step 12
                                    .splineToConstantHeading(new Vector2d(-15,40),Math.toRadians(-90)) //step 13
                                    .turn(Math.toRadians(90))
                                    .back(45)
                       /* drive.trajectorySequenceBuilder(new Pose2d(-37, -72, Math.toRadians(90))) //step 1
                                .splineTo(new Vector2d(-15,-40),Math.toRadians(90)) //step 2
                                .addDisplacementMarker(()->{}) //step 3
                                .waitSeconds(3) //step 3
                                .turn(Math.toRadians(135)) //step 4
                                .lineTo(new Vector2d(-46,-55))//step 5
                                .addDisplacementMarker(()->{}) //step 6
                                .waitSeconds(3)
                                .turn(Math.toRadians(-135)) //step 7
                                .back(10) //step 8
                                .strafeTo(new Vector2d(52,-67)) //step 8
                                .turn(Math.toRadians(-90)) //step 9
                                .addDisplacementMarker(()->{}) //step 10
                                .waitSeconds(1) //step 10
                                .turn(Math.toRadians(90)) //step 11
                                .strafeTo(new Vector2d(11,-67)) //step 12
                                .splineToConstantHeading(new Vector2d(-15,-40),Math.toRadians(90)) //step 13
                                .turn(Math.toRadians(-90))
                                .back(45)*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}