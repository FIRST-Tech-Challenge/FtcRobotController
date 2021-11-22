package org.firstinspires.ftc.Team19567.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.Team19567.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.Collections;
import java.util.List;

public class TrajectorySequence {
    private final List<SequenceSegment> sequenceList;

    public TrajectorySequence(List<SequenceSegment> sequenceList) {
        if (sequenceList.size() == 0) throw new EmptySequenceException();

        this.sequenceList = Collections.unmodifiableList(sequenceList);
    }

    public Pose2d start() {
        return sequenceList.get(0).getStartPose();
    }

    public Pose2d end() {
        return sequenceList.get(sequenceList.size() - 1).getEndPose();
    }

    public double duration() {
        double total = 0.0;

        for (SequenceSegment segment : sequenceList) {
            total += segment.getDuration();
        }

        return total;
    }

    public SequenceSegment get(int i) {
        return sequenceList.get(i);
    }

    public int size() {
        return sequenceList.size();
    }
}
