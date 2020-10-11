package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

import java.util.NoSuchElementException;

public interface RobotPathIterator {
    public class RobotPathIteratorStatus{
        public double currentDistance;
        public RobotPoint2D currentPoint;
        public RobotPathIteratorStatus(double currentDistance, RobotPoint2D currentPoint){
            this.currentDistance = currentDistance;
            this.currentPoint = currentPoint;
        }
        public RobotPathIteratorStatus(RobotPathIteratorStatus oldStatus){
            this.currentDistance = oldStatus.currentDistance;
            this.currentPoint = oldStatus.currentPoint;
        }
    }
    RobotPathIteratorStatus forward(double deltaDistance) throws NoSuchElementException;
    RobotPathIteratorStatus backward(double deltaBackwardDistance) throws NoSuchElementException;
    RobotPathIteratorStatus current();
    double getCurrentDistance();
    double getTotalDistance();
}
