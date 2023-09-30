package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;

public class TestAuto01Vis {
    enum detections {
        LEFT,
        CENTER,
        RIGHT
    }

    private static ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

    private static TrajectorySequence mergeSequences(ArrayList<TrajectorySequence> trajectorySequences) {
        TrajectorySequence[] trajectorySequencesArr = new TrajectorySequence[trajectorySequences.size()];
        trajectorySequencesArr = trajectorySequences.toArray(trajectorySequencesArr);

        return mergeSequences(trajectorySequencesArr);
    }

    private static TrajectorySequence mergeSequences(TrajectorySequence[] trajectorySequences) {
        ArrayList<SequenceSegment> trajectorySegments = new ArrayList<SequenceSegment>();

        for (TrajectorySequence sequence : trajectorySequences) {
            for (int i = 0; i < sequence.size(); i++) {
                trajectorySegments.add(sequence.get(i));
            }
        }

        return new TrajectorySequence(trajectorySegments);
    }

    private static TrajectorySequence getCurrentTrajectorySequence() {
        return sequences.get(sequences.size() - 1);
    }

    private static void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        sequences.add(trajectorySequence);
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        detections detection = detections.LEFT;
        RoadRunnerBotEntity myBot;
        DriveShim driveShim;

        myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        driveShim = myBot.getDrive();

        followTrajectorySequence(
                driveShim.trajectorySequenceBuilder(new Pose2d(61.5, -36, Math.toRadians(180)))
                        .forward(25.0)
                        .build()
        );

        assert getCurrentTrajectorySequence() != null; // Null Pointer Protection

        switch (detection) {
            case LEFT:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(getCurrentTrajectorySequence().end())
                                .forward(100.0)
                                .build()
                );
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
        }

        myBot.followTrajectorySequence(mergeSequences(sequences));

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}