package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitEndPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitHeadingInterpolationWayPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class PurePursuitWorldAxisFollower extends RobotMotionSystemTask {
    protected ArrayList<PurePursuitWayPoint> m_Path;
    protected int m_PathCursor;
    protected double m_FollowSpeedNormalized;
    protected double m_AngleSpeedNormalized;
    protected double m_PreferredAngle;
    protected double m_FollowStartSpeedNormalized;
    protected double m_SpeedThreshold = 0.05;
    protected ElapsedTime m_CurrentSegmentRuntime;
    private RobotPoint2D m_LastLookAtPoint = null;

    public PurePursuitWorldAxisFollower(List<PurePursuitWayPoint> path, double normalizedStartSpeed, double normalizedFollowSpeed, double normalizedAngleSpeed, double preferredAngle){
        this.m_Path = new ArrayList<PurePursuitWayPoint>();
        this.m_Path.addAll(path);
        this.setFollowSpeed(normalizedFollowSpeed);
        this.setStartSpeed(normalizedStartSpeed);
        this.setAngleSpeed(normalizedAngleSpeed);
        this.setPreferredAngle(preferredAngle);
    }

    public PurePursuitWorldAxisFollower(PurePursuitWorldAxisFollower oldFollower){
        super(oldFollower);
        this.m_Path = new ArrayList<PurePursuitWayPoint>();
        this.m_Path.addAll(oldFollower.m_Path);
        this.m_FollowSpeedNormalized = oldFollower.m_FollowSpeedNormalized;
        this.m_AngleSpeedNormalized = oldFollower.m_AngleSpeedNormalized;
        this.m_PreferredAngle = oldFollower.m_PreferredAngle;
        this.m_FollowStartSpeedNormalized = oldFollower.m_FollowStartSpeedNormalized;
        this.m_SpeedThreshold = oldFollower.m_SpeedThreshold;
    }

    public double getSpeedThreshold(){
        return this.m_SpeedThreshold;
    }

    public void setSpeedThreshold(double speedThreshold){
        this.m_SpeedThreshold = Math.abs(speedThreshold);
    }

    public double getStartSpeed(){
        return this.m_FollowStartSpeedNormalized;
    }

    public void setStartSpeed(double normalizedStartSpeed){
        this.m_FollowStartSpeedNormalized = Math.abs(normalizedStartSpeed);
    }

    public double getFollowSpeed(){
        return this.m_FollowSpeedNormalized;
    }

    public void setFollowSpeed(double normalizedFollowSpeed){
        this.m_FollowSpeedNormalized = Range.clip(Math.abs(normalizedFollowSpeed),0.0,1.0);
    }

    public double getAngleSpeed(){
        return this.m_AngleSpeedNormalized;
    }

    public void setAngleSpeed(double normalizedAngleSpeed){
        this.m_AngleSpeedNormalized = Range.clip(Math.abs(normalizedAngleSpeed),0.0,1.0);
    }

    public double getPreferredAngle(){
        return this.m_PreferredAngle;
    }

    public void setPreferredAngle(double preferredAngle){
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
    }

    public List<PurePursuitWayPoint> getPath(){
        return this.m_Path;
    }

    @Override
    protected void __startTask() {
        this.m_PathCursor = -1;
        this.m_CurrentSegmentRuntime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    @Override
    protected void __taskFinished() {
        this.m_CurrentSegmentRuntime = null;
    }

    @Override
    protected void __updateStatus() {
        RobotPose2D currentPosition = this.getMotionSystem().getCurrentPosition();
        PurePursuitPathFollower.FollowInformation followInfo = this.getFollowInformation(currentPosition);
        if (followInfo == null) {
            this.stopTask();
            return;
        }
        if (followInfo.purePursuitWayPoint.SegmentBeginAction != null) {
            if (!followInfo.purePursuitWayPoint.SegmentBeginAction.isActionFinished() && !followInfo.purePursuitWayPoint.SegmentBeginAction.isBusy()) {
                followInfo.purePursuitWayPoint.SegmentBeginAction.startAction();
            }
            if (followInfo.purePursuitWayPoint.SegmentBeginAction.isBusy()) {
                followInfo.purePursuitWayPoint.SegmentBeginAction.updateStatus();
            }
        }
        if (followInfo.pidEnabled) {
            this.gotoPoint_PID(currentPosition, followInfo.purePursuitWayPoint, followInfo.pursuitPoint, followInfo.normalizedFollowSpeed, this.m_AngleSpeedNormalized, this.m_PreferredAngle);
        }else{
            this.gotoPoint(currentPosition, followInfo.purePursuitWayPoint, followInfo.pursuitPoint, followInfo.normalizedFollowSpeed, this.m_AngleSpeedNormalized, this.m_PreferredAngle);
        }
        this.m_LastLookAtPoint = followInfo.pursuitPoint;
    }

    public void gotoPoint_PID(RobotPose2D currentPosition, PurePursuitWayPoint pursuitWayPoint, RobotPoint2D pursuitPoint, double pursuitSpeedNormalized, double angleSpeedNormalized, double preferredAngle){
        RobotPoint2D currentRobotAxisTarget = XYPlaneCalculations.getRelativePosition(currentPosition,pursuitPoint);
        double targetPointAngle = Math.toDegrees(Math.atan2(currentRobotAxisTarget.Y,currentRobotAxisTarget.X));
        double wantedAngleInCurrentRobotAxis = targetPointAngle + preferredAngle;
        if(pursuitWayPoint instanceof PurePursuitHeadingInterpolationWayPoint){
            double preferredAngleInOriginalRobotAxis = ((PurePursuitHeadingInterpolationWayPoint) pursuitWayPoint).getDesiredHeading();
            wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentPosition.getRotationZ();
        }else if(pursuitWayPoint instanceof PurePursuitEndPoint){
            PurePursuitEndPoint endPoint = (PurePursuitEndPoint) pursuitWayPoint;
            if(endPoint.headingInterpolationEnabled){
                double preferredAngleInOriginalRobotAxis = endPoint.getDesiredHeading();
                wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentPosition.getRotationZ();
            }
        }
        wantedAngleInCurrentRobotAxis = XYPlaneCalculations.normalizeDeg(wantedAngleInCurrentRobotAxis);
        RobotPose2D currentOffset = XYPlaneCalculations.getRelativePosition(super.getSupposedWorldTaskStartPose(),currentPosition);
        RobotVector2D velocityCorrectionVector = super.getErrorCorrectionVelocityVector(currentOffset,currentRobotAxisTarget.X,currentRobotAxisTarget.Y,wantedAngleInCurrentRobotAxis);
        this.getMotionSystem().setRobotSpeed(velocityCorrectionVector);
    }

    public void gotoPoint(RobotPose2D currentPosition, PurePursuitWayPoint pursuitWayPoint, RobotPoint2D pursuitPoint, double pursuitSpeedNormalized, double angleSpeedNormalized, double preferredAngle){
        RobotPoint2D currentRobotAxisTarget = XYPlaneCalculations.getRelativePosition(currentPosition,pursuitPoint);
        double targetPointAngle = Math.toDegrees(Math.atan2(currentRobotAxisTarget.Y,currentRobotAxisTarget.X));
        double wantedAngleInCurrentRobotAxis = targetPointAngle + preferredAngle;
        if(pursuitWayPoint instanceof PurePursuitHeadingInterpolationWayPoint){
            double preferredAngleInOriginalRobotAxis = ((PurePursuitHeadingInterpolationWayPoint) pursuitWayPoint).getDesiredHeading();
            wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentPosition.getRotationZ();
        }else if(pursuitWayPoint instanceof PurePursuitEndPoint){
            PurePursuitEndPoint endPoint = (PurePursuitEndPoint) pursuitWayPoint;
            if(endPoint.headingInterpolationEnabled){
                double preferredAngleInOriginalRobotAxis = endPoint.getDesiredHeading();
                wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentPosition.getRotationZ();
            }
        }

        wantedAngleInCurrentRobotAxis = XYPlaneCalculations.normalizeDeg(wantedAngleInCurrentRobotAxis);

        double xSpeed = currentRobotAxisTarget.X, ySpeed = currentRobotAxisTarget.Y;
        double tempSpeed = Math.abs(currentRobotAxisTarget.X) + Math.abs(currentRobotAxisTarget.Y);
        double tempFactor = pursuitSpeedNormalized / tempSpeed;
        xSpeed *= tempFactor;
        ySpeed *= tempFactor;
        double angularSpeed = Range.clip(wantedAngleInCurrentRobotAxis / 30.0,-1.0,1.0) * angleSpeedNormalized;

        this.getMotionSystem().setNormalizedRobotSpeed(
                xSpeed,
                ySpeed,
                angularSpeed
        );
    }

    public PurePursuitPathFollower.FollowInformation getFollowInformation(RobotPose2D currentPosition){

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment = false;
        double distToTarget;
        PurePursuitWayPoint target;
        DarbotsAction targetAction;
        double secondsAlreadyRunForThisSegment;
        do {
            jumpToNextSegment = false;
            target = this.m_Path.get(this.m_PathCursor + 1);
            targetAction = target.SegmentBeginAction;
            distToTarget = currentPosition.distanceTo(target);
            secondsAlreadyRunForThisSegment = this.m_CurrentSegmentRuntime.seconds();
            if (target instanceof PurePursuitEndPoint) {
                PurePursuitEndPoint targetEnd = (PurePursuitEndPoint) target;
                if (distToTarget <= targetEnd.getEndErrorToleranceDistance() && Math.abs(XYPlaneCalculations.normalizeDeg(currentPosition.getRotationZ() - targetEnd.getDesiredHeading())) <= targetEnd.getAllowedHeadingError()) {
                    jumpToNextSegment = true;
                }
            }else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
                PurePursuitHeadingInterpolationWayPoint ptTarget = (PurePursuitHeadingInterpolationWayPoint) target;
                if ((Math.abs(XYPlaneCalculations.normalizeDeg(currentPosition.getRotationZ() - ptTarget.getDesiredHeading())) <= ptTarget.getAllowedHeadingError())) {
                    jumpToNextSegment = true;
                }
            }else {
                if (distToTarget <= target.getFollowDistance()) {
                    jumpToNextSegment = true;
                }
            }

            if(jumpToNextSegment && targetAction != null){
                if(!targetAction.isActionFinished()){
                    if(targetAction.isBusy()) {
                        //the segment has been followed for at least one loop time, but the action isn't finished yet.
                        if (target.skipActionWhenSegmentFinished) {
                            if(target.stopActionWhenSkipping) {
                                targetAction.stopAction();
                            }
                        } else {
                            jumpToNextSegment = false;
                        }
                    }else{
                        //the segment is being jumped, we can force it to not jump and go to the segment.
                        if (target.skipActionWhenSegmentFinished) {
                            //we do not have to execute the action because it is going to be interrupted in the end anyway.
                        } else {
                            jumpToNextSegment = false;
                        }
                    }
                }
            }

            if(secondsAlreadyRunForThisSegment >= target.allowedSecondsForThisSegment && target.allowedSecondsForThisSegment > 0){
                if(targetAction.isBusy()){
                    if(target.stopActionWhenSkipping && targetAction != null){
                        targetAction.stopAction();
                    }
                }
                jumpToNextSegment = true;
            }

            if (jumpToNextSegment) {
                this.m_PathCursor++;
                this.m_CurrentSegmentRuntime.reset();
            }
        } while (jumpToNextSegment && this.m_PathCursor < this.m_Path.size() - 1);
        if (this.m_PathCursor >= this.m_Path.size() - 1) {return null;}

        RobotPoint2D targetStartPoint;


        //Calculate Follow Speed
        double currentSegmentEndSpeed = target.getEndFollowNormalizedSpeed();
        if(currentSegmentEndSpeed == 0){
            currentSegmentEndSpeed = this.getFollowSpeed();
        }


        double currentSegmentStartSpeed = 0;
        if(this.m_PathCursor < 0){
            targetStartPoint = this.getSupposedWorldTaskStartPose();
            currentSegmentStartSpeed = this.getStartSpeed();
        }else{
            PurePursuitWayPoint targetStart = this.m_Path.get(this.m_PathCursor);
            targetStartPoint = targetStart;
            currentSegmentStartSpeed = targetStart.getEndFollowNormalizedSpeed() == 0 ? this.getFollowSpeed() : targetStart.getEndFollowNormalizedSpeed();
        }

        double followSpeed;
        if(currentSegmentEndSpeed != currentSegmentStartSpeed) {
            double distSegmentStartEnd = target.distanceTo(targetStartPoint);
            double distToEndPoint = distToTarget;

            double progress = (distSegmentStartEnd - distToEndPoint) / distSegmentStartEnd;
            if (progress >= 0) {
                progress = Math.sqrt(progress);
            } else {
                progress = -Math.sqrt(-progress);
            }
            followSpeed = currentSegmentStartSpeed * progress + currentSegmentEndSpeed * (1.0 - progress);
        }else{
            followSpeed = currentSegmentStartSpeed;
        }
        //End Calculating Follow Speed

        //fix Follow Speed if we are too much below friction speed.
        if(followSpeed < this.m_SpeedThreshold){
            followSpeed = this.m_SpeedThreshold;
        }

        //set Follow Speed to 0 if we are done with moving but need to perform action / turn
        if(target instanceof PurePursuitEndPoint){
            if(distToTarget < ((PurePursuitEndPoint) target).getEndErrorToleranceDistance()){
                followSpeed = 0;
            }
        }else{
            if(distToTarget < Math.min(target.getFollowDistance(),5)){
                followSpeed = 0;
            }
        }

        if (target instanceof PurePursuitEndPoint && distToTarget < target.getFollowDistance()) {
            if(distToTarget <= ((PurePursuitEndPoint) target).getEndingPIDDistance()){
                return new PurePursuitPathFollower.FollowInformation(target,target,followSpeed,true);
            }else {
                return new PurePursuitPathFollower.FollowInformation(target, target, followSpeed, false);
            }
        } else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
            if(distToTarget < target.getFollowDistance()) {
                return new PurePursuitPathFollower.FollowInformation(target, target, followSpeed,false);
            }else{
                return new PurePursuitPathFollower.FollowInformation(target,getCurrentSegmentFollowPoint(currentPosition,targetStartPoint,target),followSpeed,false);
            }
        } else {
            return new PurePursuitPathFollower.FollowInformation(target,getCurrentSegmentFollowPoint(currentPosition,targetStartPoint,target),followSpeed,false);
        }
    }

    protected RobotPoint2D getCurrentSegmentFollowPoint(RobotPoint2D currentPositionPoint, RobotPoint2D startPoint, PurePursuitWayPoint endWayPoint){
        RobotPoint2D centerPoint = XYPlaneCalculations.nearestPointOnLine(currentPositionPoint,startPoint,endWayPoint);
        LinkedList<RobotPoint2D> intersectionPoints = XYPlaneCalculations.lineCircleIntersections(centerPoint,endWayPoint.getFollowDistance(),startPoint,endWayPoint);
        RobotPoint2D closestPoint = XYPlaneCalculations.getNearestPoint(intersectionPoints,endWayPoint);
        return closestPoint;
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return null;
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {
        RobotPoint2D lastPoint = this.getSupposedWorldTaskStartPose();
        for(PurePursuitWayPoint i : this.m_Path){
            dashboardCanvas.strokeLine(lastPoint.X * XYPlaneCalculations.INCH_PER_CM,lastPoint.Y * XYPlaneCalculations.INCH_PER_CM,i.X * XYPlaneCalculations.INCH_PER_CM,i.Y * XYPlaneCalculations.INCH_PER_CM);
            lastPoint = i;
        }
        if(this.m_LastLookAtPoint != null){
            dashboardCanvas.setStroke(this.getMotionSystem().supposedPoseDrawColor);
            RobotPose2D currentPosition = this.getMotionSystem().getCurrentPosition();
            dashboardCanvas.strokeLine(currentPosition.X * XYPlaneCalculations.INCH_PER_CM,currentPosition.Y * XYPlaneCalculations.INCH_PER_CM,m_LastLookAtPoint.X * XYPlaneCalculations.INCH_PER_CM, m_LastLookAtPoint.Y * XYPlaneCalculations.INCH_PER_CM);
            dashboardCanvas.setStroke(this.getMotionSystem().pathDrawColor);
        }
    }
}