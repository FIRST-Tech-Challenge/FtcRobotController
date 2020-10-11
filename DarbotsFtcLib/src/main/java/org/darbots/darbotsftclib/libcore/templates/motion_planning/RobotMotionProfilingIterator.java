package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import java.util.NoSuchElementException;

public interface RobotMotionProfilingIterator<StatusType, SegmentType> {
    StatusType forward(double deltaDuration) throws NoSuchElementException;
    StatusType backward(double deltaBackwardDuration) throws NoSuchElementException;
    StatusType current();
    SegmentType currentSegment();
    double getCurrentDuration();
    double getTotalDuration();
}
