package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.*;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

public class TrajectoryAssembly {
    private final List<TrajectoryMarker> trajectoryMarkers;
    private final List<TrajectorySegment> trajectorySegments;
    private final Log log;

    private MarkerType lastMarkerType;

    public TrajectoryAssembly(){
        trajectorySegments = new ArrayList<>();
        trajectoryMarkers = new ArrayList<>();
        log = new Log("TRAJECTORY ASSEMBLY", "start time", "marker type", "relevant contents");
    }

    public TrajectoryAssembly addSegment(Pose2D endPose2D){
        if(trajectorySegments.isEmpty()){
            this.trajectorySegments.add(new TrajectorySegment(RobotConfig.robotPose2D, endPose2D));
        }
        else{
            this.trajectorySegments.add(new TrajectorySegment(trajectorySegments.get(trajectorySegments.size()-1).endPose2D, endPose2D));
        }
        lastMarkerType = MarkerType.POSITION;
        return this;
    }

    public TrajectoryAssembly addOffsetActionMarker(double offset, MarkerAction action){
        if(!trajectorySegments.isEmpty()){
            if(lastMarkerType == MarkerType.ACTION){
                trajectoryMarkers.add(new TrajectoryMarker(trajectoryMarkers.size()-1, MarkerType.ACTION, offset, action));
            }
            else{
                trajectoryMarkers.add(new TrajectoryMarker(trajectorySegments.size()-1, MarkerType.POSITION, offset, action));
            }
        }
        else if(offset>0){
            trajectoryMarkers.add(new TrajectoryMarker(offset, action));
        }
        else{
            trajectoryMarkers.add(new TrajectoryMarker(0, action));
        }
        lastMarkerType = MarkerType.ACTION;
        return this;
    }

    private void sortMarkers(List<TrajectoryMarker> list){
        list.sort((TrajectoryMarker i1, TrajectoryMarker i2) -> {
            Double item1 = i1.getStartTime();
            Double item2 = i2.getStartTime();
            return item1.compareTo(item2);
        });
    }

    private void motionProfiling(){
        double speedUpTime = 0;
        double speedUpDistance = 0;
        Pose2D previousRobotPose2D = null;
        Angle previousModuleHeading = null;
        boolean firstSegment = true;
        double initialVelocity = 0;
        for (int i = 0; i < trajectorySegments.size(); i++) {
            double pathTime;
            double maxVelocity = trajectorySegments.get(i).velocity;
            if(!firstSegment && (i == trajectorySegments.size() - 1)){
                //last segment stuff here

                double slowDownTime = (maxVelocity)/ RobotConstants.maxSwerveAcceleration;
                double slowDownDistance = (maxVelocity)*0.5*slowDownTime;
                double retainedVelocity = 0;

                double robotTurnTime = ((previousRobotPose2D.angle.angleShortDifference(trajectorySegments.get(i).endPose2D.angle)/90)* RobotConstants.robotHeading_kP* RobotConstants.robotMaxAngularVelocity* trajectorySegments.get(i).turnPower*0.5);

                if(trajectorySegments.get(i).distance < slowDownDistance + speedUpDistance){
                    speedUpDistance = (trajectorySegments.get(i).distance / (speedUpDistance + slowDownDistance)) * speedUpDistance;
                    slowDownDistance = (trajectorySegments.get(i).distance / (speedUpDistance + slowDownDistance)) * slowDownDistance;

                    double newMaxVelocity = Math.sqrt(initialVelocity*initialVelocity + 2 * RobotConstants.maxSwerveAcceleration * speedUpDistance);
                    speedUpTime = (newMaxVelocity-initialVelocity)/ RobotConstants.maxSwerveAcceleration;
                    slowDownTime = (newMaxVelocity-retainedVelocity)/ RobotConstants.maxSwerveAcceleration;

                    pathTime = speedUpTime + slowDownTime;
                }
                else {
                    pathTime = speedUpTime + slowDownTime + (trajectorySegments.get(i).distance - speedUpDistance - slowDownDistance) / maxVelocity;
                }

                pathTime += robotTurnTime;

                //instruction to target new pose
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime() + pathTime, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to slow down
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime() - slowDownTime, 0));

                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime() + slowDownTime, () -> RobotConfig.currentTrajectoryAssembly = null));
            }
            else if (!firstSegment) {
                double angleChange = previousModuleHeading.angleShortDifference(Angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D));
                double retainedVelocity;
                if(angleChange < 90 || previousRobotPose2D.angle.getValue() == trajectorySegments.get(i).endPose2D.angle.getValue()){
                    retainedVelocity = Math.cos(angleChange)* RobotConstants.maxSwerveVelocity;
                }
                else {
                    retainedVelocity = 0;
                }
                double slowDownTime = (maxVelocity - retainedVelocity)/ RobotConstants.maxSwerveAcceleration;
                double slowDownDistance = (maxVelocity - retainedVelocity)*0.5*slowDownTime;


                double robotTurnTime = ((previousRobotPose2D.angle.angleShortDifference(trajectorySegments.get(i).endPose2D.angle)/90)* RobotConstants.robotHeading_kP* RobotConstants.robotMaxAngularVelocity* trajectorySegments.get(i).turnPower*0.5);
                double moduleTurnTime = ((previousModuleHeading.angleShortDifference(Angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D))/90)* RobotConstants.moduleAngle_kP* RobotConstants.moduleMaxAngularVelocity*0.5);

                if(trajectorySegments.get(i).distance < slowDownDistance + speedUpDistance){
                    speedUpDistance = (trajectorySegments.get(i).distance / (speedUpDistance + slowDownDistance)) * speedUpDistance;
                    slowDownDistance = (trajectorySegments.get(i).distance / (speedUpDistance + slowDownDistance)) * slowDownDistance;

                    double newMaxVelocity = Math.sqrt(initialVelocity*initialVelocity + 2 * RobotConstants.maxSwerveAcceleration * speedUpDistance);
                    speedUpTime = (newMaxVelocity-initialVelocity)/ RobotConstants.maxSwerveAcceleration;
                    slowDownTime = (newMaxVelocity-retainedVelocity)/ RobotConstants.maxSwerveAcceleration;

                    pathTime = speedUpTime + slowDownTime;
                }
                else {
                    pathTime = speedUpTime + slowDownTime + (trajectorySegments.get(i).distance - speedUpDistance - slowDownDistance) / maxVelocity;
                }

                pathTime += robotTurnTime;

                //instruction to target new pose
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime()+pathTime, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to slow down
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime()-slowDownTime, retainedVelocity));

                //instruction to accelerate to max velocity
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime(), maxVelocity));

                speedUpTime = (maxVelocity - retainedVelocity)/ RobotConstants.maxSwerveAcceleration;
                speedUpDistance = (maxVelocity - retainedVelocity)*0.5*speedUpTime;
                initialVelocity = retainedVelocity;
            }
            else {
                //first segment stuff here

                //instruction to target new pose
                trajectoryMarkers.add(new TrajectoryMarker(0, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to accelerate to max velocity
                trajectoryMarkers.add(new TrajectoryMarker(getLastTagType(MarkerType.POSITION).getStartTime(), maxVelocity));

                speedUpTime = (maxVelocity)/ RobotConstants.maxSwerveAcceleration;
                speedUpDistance = (maxVelocity)*0.5*speedUpTime;
                initialVelocity = 0;

                firstSegment = false;
            }
            previousRobotPose2D = trajectorySegments.get(i).endPose2D;
            previousModuleHeading = Angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D);
        }
    }

    private void actionMarkerTimeFinalising(){
        List<Integer> actionMarkerlist = new ArrayList<>();
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            if(trajectoryMarkers.get(i).getMarkerType() == MarkerType.ACTION){
                actionMarkerlist.add(i);
            }
        }
        for (int i = 0; i < actionMarkerlist.size(); i++) {
            double startTime = getIndexOfTagType(trajectoryMarkers.get(actionMarkerlist.get(i)).getReferenceType(), trajectoryMarkers.get(actionMarkerlist.get(i)).getReferenceIndex()).getStartTime();
            trajectoryMarkers.get(actionMarkerlist.get(i)).setStartTime(startTime+trajectoryMarkers.get(actionMarkerlist.get(i)).getStartTime());
        }
    }

    public TrajectoryAssembly build(){
        motionProfiling();
        actionMarkerTimeFinalising();
        sortMarkers(trajectoryMarkers);
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            log.logData(0, trajectoryMarkers.get(i).getStartTime());
            log.logData(1, trajectoryMarkers.get(i).getMarkerType());
            log.logData(2, trajectoryMarkers.get(i).getMarkerAction().toString());
            log.updateLoop(true);
        }
        log.close();
        return this;
    }

    public void update(){
        if(RobotConfig.currentTrajectoryAssembly.trajectoryMarkers.isEmpty()){ //TODO fix up to ensure that the end of a path is reached before continuing to the next one, maybe add a nullification action to the end of the markers list during build instead
            RobotConfig.currentTrajectoryAssembly = null;
            return;
        }
        if (!(RobotConfig.currentTrajectoryAssembly.trajectoryMarkers.get(0).getStartTime() <= RobotConfig.elapsedTime.seconds())) {
            return;
        }
        RobotConfig.currentTrajectoryAssembly.trajectoryMarkers.get(0).getMarkerAction().markerReached();
        RobotConfig.currentTrajectoryAssembly.trajectoryMarkers.remove(0);
    }

    public boolean isBusy(){
        return !(RobotConfig.currentTrajectoryAssembly == null);
    }

    public TrajectoryMarker getLastTagType(MarkerType markerType){
        List<Integer> list = new ArrayList<>();
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            if(trajectoryMarkers.get(i).getMarkerType() == markerType){
                list.add(i);
            }
        }
        return trajectoryMarkers.get(list.get(list.size()-1));
    }

    public TrajectoryMarker getIndexOfTagType(MarkerType markerType, int index){
        List<Integer> list = new ArrayList<>();
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            if(trajectoryMarkers.get(i).getMarkerType() == markerType){
                list.add(i);
            }
        }
        if(index<0){
            return trajectoryMarkers.get(list.get(0));
        }
        else if(index>(list.size()-1)){
            return trajectoryMarkers.get(list.get(list.size()-1));
        }
        else{
            return trajectoryMarkers.get(list.get(index));
        }
    }
}
