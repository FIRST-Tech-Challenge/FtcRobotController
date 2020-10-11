package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public abstract class RobotBasic2DPositionTracker implements Robot2DPositionTracker {
    protected RobotPose2D m_InitialPos;
    protected RobotPose2D m_CurrentPos;
    protected RobotVector2D m_VelocityVector;

    public RobotBasic2DPositionTracker(RobotPose2D initialPosition){
        this.m_InitialPos = new RobotPose2D(initialPosition);
        this.m_CurrentPos = new RobotPose2D(initialPosition);
        this.m_VelocityVector = new RobotPose2D(0,0,0);
    }

    public RobotBasic2DPositionTracker(Robot2DPositionTracker oldTracker){
        this.m_InitialPos = new RobotPose2D(oldTracker.getInitialPos());
        this.m_CurrentPos = new RobotPose2D(oldTracker.getCurrentPosition());
        this.m_VelocityVector = new RobotPose2D(oldTracker.getCurrentVelocityVector());
    }

    @Override
    public RobotPose2D getInitialPos(){
        return this.m_InitialPos;
    }

    public void setInitialPos(RobotPose2D initialPos){
        this.m_InitialPos.X = initialPos.X;
        this.m_InitialPos.Y = initialPos.Y;
        this.m_InitialPos.setRotationZ(initialPos.getRotationZ());
    }

    @Override
    public RobotPose2D getCurrentPosition(){
        return this.m_CurrentPos;
    }

    public void setCurrentPosition(RobotPose2D currentPosition){
        this.m_CurrentPos.X = currentPosition.X;
        this.m_CurrentPos.Y = currentPosition.Y;
        this.m_CurrentPos.setRotationZ(currentPosition.getRotationZ());
    }

    @Override
    public RobotPose2D fieldAxisFromRobotAxis(RobotPose2D RobotAxisPoint){
        return XYPlaneCalculations.getAbsolutePosition(this.getCurrentPosition(),RobotAxisPoint);
    }
    @Override
    public RobotPose2D robotAxisFromFieldAxis(RobotPose2D FieldAxisPoint){
        return XYPlaneCalculations.getRelativePosition(this.getCurrentPosition(),FieldAxisPoint);
    }

    protected void offsetPosition(RobotPose2D offsetPosition) {
        RobotPose2D currentPosition = this.m_CurrentPos;
        if(offsetPosition.X != 0)
            currentPosition.X += offsetPosition.X;
        if(offsetPosition.Y != 0)
            currentPosition.Y += offsetPosition.Y;
        if(offsetPosition.getRotationZ() != 0)
            currentPosition.setRotationZ(currentPosition.getRotationZ() + offsetPosition.getRotationZ());
    }

    @Override
    public RobotVector2D getCurrentVelocityVector(){
        return this.m_VelocityVector;
    }

    public void setCurrentVelocityVector(RobotVector2D velocityVector){
        this.m_VelocityVector.X = velocityVector.X;
        this.m_VelocityVector.Y = velocityVector.Y;
        this.m_VelocityVector.setRotationZ(velocityVector.getRotationZ());
    }
}
