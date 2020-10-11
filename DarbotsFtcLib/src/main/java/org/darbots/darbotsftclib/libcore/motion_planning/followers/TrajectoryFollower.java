package org.darbots.darbotsftclib.libcore.motion_planning.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.TrajectoryMotionState;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;

public class TrajectoryFollower extends RobotMotionSystemTask {
    private RobotTrajectory m_Trajectory;
    private RobotMotionProfilingIterator<TrajectoryMotionState,?> m_TrajectoryIterator;
    private double m_TotalDuration;
    private double m_AngleError = 0;
    private TrajectoryMotionState m_EndState;

    public TrajectoryFollower(RobotTrajectory trajectory){
        super();
        this.m_Trajectory = trajectory;
        this.m_EndState = new TrajectoryMotionState(0,0,0,0,0);
        this.__updateTrajectory();
    }

    public TrajectoryFollower(TrajectoryFollower oldFollower){
        super(oldFollower);
        this.m_Trajectory = oldFollower.m_Trajectory;
        this.m_EndState = new TrajectoryMotionState(0,0,0,0,0);
        this.__updateTrajectory();
    }

    protected void __updateTrajectory(){
        TrajectoryMotionState endMotionState = this.m_Trajectory.getEndState();
        this.m_EndState.setValues(endMotionState);
    }

    @Override
    protected void __startTask() {
        this.m_TrajectoryIterator = m_Trajectory.getIterator();
        this.m_TotalDuration = this.m_TrajectoryIterator.getTotalDuration();
        this.m_AngleError = 0;
    }

    @Override
    protected void __taskFinished() {
        this.m_TrajectoryIterator = null;
    }

    @Override
    protected void __updateStatus() {
        double currentTime = this.getSecondsSinceTaskStart();
        if(currentTime >= this.m_TotalDuration){
            this.stopTask();
            return;
        }
        TrajectoryMotionState supposedMotionState = this.m_TrajectoryIterator.forward(currentTime - this.m_TrajectoryIterator.getCurrentDuration());
        RobotPose2D supposedDistancePose = new RobotPose2D(supposedMotionState.xDisplacement,supposedMotionState.yDisplacement,supposedMotionState.getPreferredAngle());

        RobotPose2D actualRelativeOffset = this.getRelativePositionOffsetSinceStart();
        RobotVector2D correctionPose = this.getErrorCorrectionVelocityVector(supposedDistancePose,actualRelativeOffset);
        this.m_AngleError = XYPlaneCalculations.normalizeDeg(actualRelativeOffset.getRotationZ() - supposedMotionState.getPreferredAngle());

        double[] originalRobotAxisSpeedVector = {supposedMotionState.xVelocity,supposedMotionState.yVelocity};
        //Since supposedMotionState is in respect to the original robot axis, we should convert it to current robotaxis.
        double[] actualSpeedVector = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(originalRobotAxisSpeedVector,XYPlaneCalculations.ORIGIN_POINT_ARRAY,-actualRelativeOffset.getRotationZ());

        RobotVector2D afterCorrectionVelocity = new RobotPose2D(actualSpeedVector[0]+correctionPose.X,actualSpeedVector[1]+correctionPose.Y,0);
        RobotVector2D motionSystemMaxVelocity = this.getMotionSystem().getTheoreticalMaximumMotionState(afterCorrectionVelocity);
        RobotVector2D actualVelocity = null;
        if(Math.abs(afterCorrectionVelocity.X) > Math.abs(motionSystemMaxVelocity.X) || Math.abs(afterCorrectionVelocity.Y) > Math.abs(motionSystemMaxVelocity.Y)){
            actualVelocity = new RobotVector2D(motionSystemMaxVelocity);
        }else{
            actualVelocity = new RobotVector2D(afterCorrectionVelocity);
        }
        double possibleMaxZRot = this.getMotionSystem().calculateMaxAngularSpeedInDegPerSec(actualVelocity.X,actualVelocity.Y);
        double rotZCorrectionVelocity = Range.clip(correctionPose.getRotationZ(),-possibleMaxZRot,possibleMaxZRot);
        actualVelocity.setRotationZ(rotZCorrectionVelocity);
        this.getMotionSystem().setRobotSpeed(actualVelocity);
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        TrajectoryMotionState endMotionState = this.m_EndState;
        if(Math.abs(this.m_AngleError) <= 5){
            return new RobotPose2D(endMotionState.xDisplacement,endMotionState.yDisplacement,endMotionState.getPreferredAngle());
        }else{
            return new RobotPose2D(endMotionState.xDisplacement,endMotionState.yDisplacement,Double.NaN);
        }
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {

    }
}
