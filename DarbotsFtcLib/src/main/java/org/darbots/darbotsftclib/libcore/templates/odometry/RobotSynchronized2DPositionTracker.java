package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;

public abstract class RobotSynchronized2DPositionTracker extends RobotBasic2DPositionTracker {

    public RobotSynchronized2DPositionTracker(RobotPose2D initialPosition) {
        super(initialPosition);
    }

    public RobotSynchronized2DPositionTracker(Robot2DPositionTracker oldTracker) {
        super(oldTracker);
    }

    public RobotPose2D getInitialPos(){
        synchronized (super.m_InitialPos){
            return new RobotPose2D(super.m_InitialPos);
        }
    }
    public void setInitialPos(RobotPose2D initialPos){
        synchronized (super.m_InitialPos) {
            super.m_InitialPos = initialPos;
        }
    }
    public RobotPose2D getCurrentPosition(){
        synchronized (super.m_CurrentPos) {
            return new RobotPose2D(super.m_CurrentPos);
        }
    }
    public void setCurrentPosition(RobotPose2D currentPosition){
        RobotPose2D read2DPosition = super.m_CurrentPos;
        synchronized (read2DPosition) {
            read2DPosition.X = currentPosition.X;
            read2DPosition.Y = currentPosition.Y;
            read2DPosition.setRotationZ(currentPosition.getRotationZ());
        }
    }

    public void offsetPosition(RobotPose2D offsetPosition) {
        RobotPose2D read2DPosition = super.m_CurrentPos;
        synchronized (read2DPosition) {
            super.offsetPosition(offsetPosition);
        }
    }

    @Override
    public RobotVector2D getCurrentVelocityVector(){
        RobotVector2D readVelocityVector = super.m_VelocityVector;
        synchronized (readVelocityVector) {
            return new RobotPose2D(readVelocityVector);
        }
    }

    public void setCurrentVelocityVector(RobotVector2D velocityVector){
        RobotVector2D readVelocityVector = super.m_VelocityVector;
        synchronized (readVelocityVector){
            readVelocityVector.X = velocityVector.X;
            readVelocityVector.Y = velocityVector.Y;
            readVelocityVector.setRotationZ(velocityVector.getRotationZ());
        }
    }
}
