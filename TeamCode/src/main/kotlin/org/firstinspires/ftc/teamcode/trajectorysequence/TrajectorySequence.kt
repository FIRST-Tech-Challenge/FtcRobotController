package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment

class TrajectorySequence(sequenceList: List<SequenceSegment>) {
    private val sequenceList: List<SequenceSegment>

    init {
        if (sequenceList.isEmpty()) throw EmptySequenceException()

        this.sequenceList = java.util.Collections.unmodifiableList(sequenceList)
    }

    fun start(): Pose2d {
        return sequenceList[0].startPose
    }

    fun end(): Pose2d {
        return sequenceList[sequenceList.size - 1].endPose
    }

    fun duration(): Double {
        var total = 0.0

        for (segment in sequenceList) {
            total += segment.duration
        }

        return total
    }

    fun get(i: Int): SequenceSegment {
        return sequenceList[i]
    }

    fun size(): Int {
        return sequenceList.size
    }
}