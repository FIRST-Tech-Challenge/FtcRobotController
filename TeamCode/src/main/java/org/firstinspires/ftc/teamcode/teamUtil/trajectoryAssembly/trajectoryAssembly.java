package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.*;

import static org.apache.commons.math3.util.Precision.compareTo;

import androidx.appcompat.widget.ActionBarOverlayLayout;

import java.util.ArrayList;
import java.util.List;

public class trajectoryAssembly {
    private final List<trajectoryMarker> trajectoryMarkers;
    private final List<trajectorySegment> trajectorySegments;
    private final log log;

    public trajectoryAssembly(){
        trajectorySegments = new ArrayList<>();
        trajectoryMarkers = new ArrayList<>();
        log = new log("TRAJECTORY ASSEMBLY", "start time", "marker type", "relevant contents");
    }

    public trajectoryAssembly addSegment(pose2D endPose2D){
        if(trajectorySegments.isEmpty()){
            this.trajectorySegments.add(new trajectorySegment(robotConfig.robotPose2D, endPose2D));
        }
        else{
            this.trajectorySegments.add(new trajectorySegment(trajectorySegments.get(trajectorySegments.size()-1).endPose2D, endPose2D));
        }
        return this;
    }

    public trajectoryAssembly addSegment(pose2D endPose2D, double turnPower){
        if(turnPower > 1){
            turnPower = 1;
        }
        else if(turnPower < 0) {
            turnPower = 0;
        }
        if(trajectorySegments.isEmpty()){
            this.trajectorySegments.add(new trajectorySegment(robotConfig.robotPose2D, endPose2D, turnPower));
        }
        else{
            this.trajectorySegments.add(new trajectorySegment(trajectorySegments.get(trajectorySegments.size()-1).endPose2D, endPose2D, turnPower));
        }
        return this;
    }

    public trajectoryAssembly addOffsetActionMarker(double offset, markerAction action){
        if(!trajectorySegments.isEmpty()){
            trajectoryMarkers.add(new trajectoryMarker(trajectorySegments.size()-1, offset, action));
        }
        else if(offset>0){
            trajectoryMarkers.add(new trajectoryMarker(offset, action));
        }
        else{
            trajectoryMarkers.add(new trajectoryMarker(0, action));
        }
        return this;
    }

    void sortMarkers(List<trajectoryMarker> list){
        list.sort((trajectoryMarker i1, trajectoryMarker i2) -> compareTo(i1.getStartTime(), i2.getStartTime(), 0.001));
    }

    private void motionProfiling(){
        double speedUpTime = 0;
        double speedUpDistance = 0;
        pose2D previousRobotPose2D = null;
        angle previousModuleHeading = null;
        boolean firstSegment = true;
        for (int i = 1; i < trajectorySegments.size(); i++) {
            double pathTime;
            double maxVelocity = trajectorySegments.get(i).velocity;
            if(!firstSegment && (i == trajectorySegments.size()-1)){
                //last segment stuff here

                double slowDownTime = (maxVelocity)/robotConstants.maxSwerveAcceleration;
                double slowDownDistance = (maxVelocity)*0.5*slowDownTime;

                if(trajectorySegments.get(i).distance < slowDownDistance + speedUpDistance){
                    speedUpTime = (speedUpDistance / (speedUpDistance - (((slowDownDistance + speedUpDistance) - trajectorySegments.get(i).distance) * 0.5))) * speedUpTime;
                    slowDownTime = (slowDownDistance / (slowDownDistance - (((slowDownDistance + speedUpDistance) - trajectorySegments.get(i).distance) * 0.5))) * slowDownTime;

                    pathTime = speedUpTime + slowDownTime;
                }
                else {
                    pathTime = speedUpTime + slowDownTime + (trajectorySegments.get(i).distance - speedUpDistance - slowDownDistance) / maxVelocity;
                }

                //instruction to target new pose and then at the same time do the heading handling TODO: remove heading handling
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime()+pathTime, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to slow down
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime()-slowDownTime, 0));

            }
            else if (!firstSegment) {
                double angleChange = previousModuleHeading.angleShortDifference(angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D));
                double retainedVelocity;
                if(angleChange < 90){
                    retainedVelocity = Math.cos(angleChange)*robotConstants.maxSwerveVelocity;
                }
                else {
                    retainedVelocity = 0;
                }
                double slowDownTime = (maxVelocity - retainedVelocity)/robotConstants.maxSwerveAcceleration;
                double slowDownDistance = (maxVelocity - retainedVelocity)*0.5*slowDownTime;

                /*
                double robotTurnTime = 0;// ((previousRobotPose2D.angle.angleShortDifference(trajectorySegments.get(i).endPose2D.angle)/90)*robotConstants.robotHeading_kP*robotConstants.robotMaxAngularVelocity*trajectorySegments.get(i).turnPower*0.5);
                double moduleTurnTime =  0;// ((previousModuleHeading.angleShortDifference(angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D))/90)*robotConstants.moduleAngle_kP*robotConstants.moduleMaxAngularVelocity*0.5);
                */
                if(trajectorySegments.get(i).distance < slowDownDistance + speedUpDistance){
                    speedUpTime = (speedUpDistance / (speedUpDistance - (((slowDownDistance + speedUpDistance) - trajectorySegments.get(i).distance) * 0.5))) * speedUpTime;
                    slowDownTime = (slowDownDistance / (slowDownDistance - (((slowDownDistance + speedUpDistance) - trajectorySegments.get(i).distance) * 0.5))) * slowDownTime;

                    pathTime = speedUpTime + slowDownTime;
                }
                else {
                    pathTime = speedUpTime + slowDownTime + (trajectorySegments.get(i).distance - speedUpDistance - slowDownDistance) / maxVelocity;
                }

                //instruction to target new pose and then at the same time do the heading handling TODO: remove heading handling
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime()+pathTime, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to slow down
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime()-slowDownTime, retainedVelocity));

                //instruction to accelerate to max velocity
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime(), maxVelocity));

                speedUpTime = (maxVelocity - retainedVelocity)/robotConstants.maxSwerveAcceleration;
                speedUpDistance = (maxVelocity - retainedVelocity)*0.5*speedUpTime;
            }
            else {
                //first segment stuff here

                //instruction to target new pose and then at the same time do the heading handling TODO: remove heading handling
                trajectoryMarkers.add(new trajectoryMarker(0, trajectorySegments.get(i).endPose2D, trajectorySegments.get(i).turnPower));

                //instruction to accelerate to max velocity
                trajectoryMarkers.add(new trajectoryMarker(getLastTagType(markerType.POSITION).getStartTime(), maxVelocity));

                speedUpTime = (maxVelocity)/robotConstants.maxSwerveAcceleration;
                speedUpDistance = (maxVelocity)*0.5*speedUpTime;

                firstSegment = false;
            }
            previousRobotPose2D = trajectorySegments.get(i).endPose2D;
            previousModuleHeading = angle.atanHandler(trajectorySegments.get(i).endPose2D, previousRobotPose2D);
        }
    }

    private void actionMarkerTimeFinalising(){
        List<Integer> list = new ArrayList<>();
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            if(trajectoryMarkers.get(i).getMarkerType() == markerType.ACTION){
                list.add(i);
            }
        }
        for (int i = 0; i < list.size(); i++) {
            double startTime = getIndexOfTagType(markerType.POSITION, trajectoryMarkers.get(list.get(i)).getReferenceIndex()).getStartTime();
            trajectoryMarkers.get(list.get(i)).setStartTime(startTime+trajectoryMarkers.get(list.get(i)).getStartTime());
        }
    }

    public trajectoryAssembly build(){
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
        if(robotConfig.currentTrajectoryAssembly.trajectoryMarkers.isEmpty()){
            robotConfig.currentTrajectoryAssembly = null;
            return;
        }
        if (!(robotConfig.currentTrajectoryAssembly.trajectoryMarkers.get(0).getStartTime() <= robotConfig.elapsedTime.seconds())) {
            return;
        }
        robotConfig.currentTrajectoryAssembly.trajectoryMarkers.get(0).getMarkerAction().markerReached();
        robotConfig.currentTrajectoryAssembly.trajectoryMarkers.remove(0);
    }

    public boolean isBusy(){
        return !(robotConfig.currentTrajectoryAssembly == null);
    }

    public trajectoryMarker getLastTagType(markerType markerType){
        List<Integer> list = new ArrayList<>();
        for (int i = 0; i < trajectoryMarkers.size(); i++) {
            if(trajectoryMarkers.get(i).getMarkerType() == markerType){
                list.add(i);
            }
        }
        return trajectoryMarkers.get(list.get(list.size()-1));
    }

    public trajectoryMarker getIndexOfTagType(markerType markerType, int index){
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
