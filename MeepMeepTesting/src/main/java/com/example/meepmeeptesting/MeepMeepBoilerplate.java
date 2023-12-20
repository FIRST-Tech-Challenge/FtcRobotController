package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;

enum Detection {
    LEFT,
    CENTER,
    RIGHT
}

public abstract class MeepMeepBoilerplate {
    DriveShim drive;

    public Pose2d STARTING_POSE = new Pose2d(-36, 61.5, Math.toRadians(-90));

    private MeepMeep meepMeep;

    private RoadRunnerBotEntity myBot;

    public ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

    public void drive(Pose2d startingPos) {
        STARTING_POSE = startingPos;
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        drive = myBot.getDrive();

        Detection detection = detect();

        TrajectorySequence trajectorySequence = getTrajectorySequence(detection, drive);

        myBot.followTrajectorySequence(trajectorySequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public Detection detect() {
        return Detection.RIGHT;
    }

    public abstract TrajectorySequence getTrajectorySequence(Detection detection, DriveShim drive);

    public TrajectorySequence mergeSequences(ArrayList<TrajectorySequence> trajectorySequences) {
        TrajectorySequence[] trajectorySequencesArr = new TrajectorySequence[trajectorySequences.size()];
        trajectorySequencesArr = trajectorySequences.toArray(trajectorySequencesArr);

        return mergeSequences(trajectorySequencesArr);
    }

    public TrajectorySequence mergeSequences(TrajectorySequence[] trajectorySequences) {
        ArrayList<SequenceSegment> trajectorySegments = new ArrayList<SequenceSegment>();

        for (TrajectorySequence sequence : trajectorySequences) {
            for (int i = 0; i < sequence.size(); i++) {
                trajectorySegments.add(sequence.get(i));
            }
        }

        return new TrajectorySequence(trajectorySegments);
    }

    public TrajectorySequence getCurrentTrajectorySequence(DriveShim drive) {
         if (sequences.size() == 0) {
             return drive.trajectorySequenceBuilder(STARTING_POSE)
                    .forward(0.0)
                    .build();
        } else {
            return sequences.get(sequences.size() - 1);
        }
    }

    public Pose2d getCurrentPosition(DriveShim drive) {
        if (sequences.size() == 0) {
            return STARTING_POSE;
        } else {
            return getCurrentTrajectorySequence(drive).end();
        }
    }

    //    private static void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//        sequences.add(trajectorySequence);
//    }
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//        if (!opModeIsActive()) waitForStart();
        sequences.add(trajectorySequence);
    }
}
