package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;

public class MeepMeepBoilerplate extends DriveMethods {
    SampleMecanumDrive drive;

    public final Pose2d STARTING_POSE = new Pose2d(-36, 61.5, Math.toRadians(-90));

    public ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

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

    public TrajectorySequence getCurrentTrajectorySequence(SampleMecanumDrive drive) {
        return sequences.get(sequences.size() - 1);
    }

    //    private static void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//        sequences.add(trajectorySequence);
//    }
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        if (!opModeIsActive()) waitForStart();
        drive.followTrajectorySequence(trajectorySequence);
    }
}
