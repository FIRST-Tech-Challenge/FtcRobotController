package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfile;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfileGenerator;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfileIterator;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionState;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class SimpleTrajectoryGenerator {
    public static SimpleTrajectory generateTrajectory(double resolutionInSeconds, MotionSystemConstraints constraints, RobotPath pathToFollow, double startSpeed, double cruiseSpeed, double endSpeed, double preferredAngle){
        preferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
        resolutionInSeconds = Math.abs(resolutionInSeconds);
        MotionProfile generatedMotionProfile = MotionProfileGenerator.generatePathMotionProfile(constraints,pathToFollow,startSpeed,cruiseSpeed,endSpeed);
        SimpleTrajectory TrajectoryResult = new SimpleTrajectory(resolutionInSeconds);

        double totalDuration = generatedMotionProfile.getTotalDuration();
        double totalDistance = pathToFollow.getTotalDistance();

        //keep adding trajectory segments to the TrajectoryResult List, by using resolution specified by the user.
        double durationCounter = 0;
        double distanceCounter = 0;
        double secondsToGo = resolutionInSeconds;
        MotionProfileIterator linearSpeedIterator = new MotionProfileIterator(generatedMotionProfile);
        while(durationCounter < totalDuration){
            //Step 1: Measure the duration of this segment.
            if(durationCounter + resolutionInSeconds > totalDuration){
                secondsToGo = totalDuration - durationCounter;
            }else{
                secondsToGo = resolutionInSeconds;
            }
            //Step 2: measure average linear velocity
            //Let's cut the piece we need from the Motion Profile.
            MotionState pieceStartLinearState = new MotionState(linearSpeedIterator.current());
            MotionState pieceEndLinearState = new MotionState(linearSpeedIterator.forward(secondsToGo));
            if(pieceStartLinearState.distance > totalDistance){
                pieceStartLinearState.distance = totalDistance;
            }
            if(pieceEndLinearState.distance > totalDistance){
                pieceEndLinearState.distance = totalDistance;
            }
            double pieceApproximateAcceleration = (pieceEndLinearState.velocity - pieceStartLinearState.velocity) / secondsToGo;

            RobotPoint2D pieceStartPoint = pathToFollow.getPointAtDistance(distanceCounter);
            RobotPoint2D pieceEndPoint = pathToFollow.getPointAtDistance(pieceEndLinearState.distance);
            double pieceDeltaX = pieceEndPoint.X - pieceStartPoint.X;
            double pieceDeltaY = pieceEndPoint.Y - pieceStartPoint.Y;
            double pieceApproximateDistance = pieceStartPoint.distanceTo(pieceEndPoint);
            double pieceApproximateAvgVelocity = pieceApproximateDistance / secondsToGo;
            double pieceApproximateStartVelocity = pieceApproximateAvgVelocity - pieceApproximateAcceleration * (secondsToGo / 2.0);
            double pieceApproximateEndVelocity = pieceApproximateAvgVelocity + pieceApproximateAcceleration * (secondsToGo / 2.0);

            //Step 3: measure average robot velocity
            //Step 3.1: measure x-y velocity proportion.
            double tempDeltaXYSpeed = Math.sqrt(Math.pow(pieceDeltaX,2) + Math.pow(pieceDeltaY,2));
            double startSpeedXYFactor = tempDeltaXYSpeed > 0 ? pieceApproximateStartVelocity / tempDeltaXYSpeed : 0;
            double endSpeedXYFactor = tempDeltaXYSpeed > 0 ? pieceApproximateEndVelocity / tempDeltaXYSpeed : 0;

            //Step 3.2: construct x-y velocity based on proportion.
            double[] pieceApproximateStartXYSpeed = {pieceDeltaX * startSpeedXYFactor, pieceDeltaY * startSpeedXYFactor};
            double[] pieceApproximateEndXYSpeed = {pieceDeltaX * endSpeedXYFactor, pieceDeltaY * endSpeedXYFactor};

            //Step 3.3: measure x-y acceleration.
            double pieceApproximateXAcceleration = (pieceApproximateEndXYSpeed[0] - pieceApproximateStartXYSpeed[0]) / secondsToGo;
            double pieceApproximateYAcceleration = (pieceApproximateEndXYSpeed[1] - pieceApproximateStartXYSpeed[1]) / secondsToGo;

            //Step 4: construct trajectory motion segment
            TrajectoryMotionSegment currentSegment = new TrajectoryMotionSegment(
                    pieceStartPoint.X,
                    pieceStartPoint.Y,
                    pieceApproximateStartXYSpeed[0],
                    pieceApproximateEndXYSpeed[1],
                    pieceApproximateXAcceleration,
                    pieceApproximateYAcceleration,
                    preferredAngle,
                    secondsToGo
            );
            TrajectoryResult.getMotionSegments().add(currentSegment);

            //Step 5: update both counters
            durationCounter += secondsToGo;
            distanceCounter = pieceEndLinearState.distance;
            //Because iterator has already been pushed forward, no need to update it.
        }
        return TrajectoryResult;
    }
}
