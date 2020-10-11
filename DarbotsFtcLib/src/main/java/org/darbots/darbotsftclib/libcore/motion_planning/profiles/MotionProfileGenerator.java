package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class MotionProfileGenerator {
    public static final double PATH_DISTANCE_ERROR_MARGIN = 0.01;
    public static MotionProfile generatePathMotionProfile(MotionSystemConstraints constraints, RobotPath Path, double startVelocity, double cruiseVelocity, double endVelocity){
        MotionProfile generatedProfile = generateMotionProfile(constraints.maximumLinearSpeed,constraints.maximumLinearAcceleration,constraints.maximumLinearJerk,Path.getTotalDistance(),startVelocity,cruiseVelocity,endVelocity);
        return generatedProfile;
    }
    public static MotionProfile generateAngularMotionProfile(MotionSystemConstraints constraints, double degToTurn, double startVelocity, double cruiseVelocity, double endVelocity){
        MotionProfile generatedProfile;
        if(degToTurn < 0){
            generatedProfile = generateMotionProfile(constraints.maximumAngularSpeed,constraints.maximumAngularAcceleration,constraints.maximumAngularJerk,-degToTurn,startVelocity,cruiseVelocity,endVelocity).negative();
        }else{
            generatedProfile = generateMotionProfile(constraints.maximumAngularSpeed,constraints.maximumAngularAcceleration,constraints.maximumAngularJerk,degToTurn,startVelocity,cruiseVelocity,endVelocity);
        }
        return generatedProfile;
    }
    public static MotionProfile generateMotionProfile(double maxVelocity, double maxAcceleration, double maxJerk, double PathTotalDistance, double startVelocity, double cruiseVelocity, double endVelocity){
        return generateMotionProfile_JERKUNLIMITED(maxVelocity,maxAcceleration,PathTotalDistance,startVelocity,cruiseVelocity,endVelocity);

        /*
        if(PathTotalDistance < 0){
            return generateMotionProfile(maxVelocity,maxAcceleration,maxJerk,-PathTotalDistance,startVelocity,cruiseVelocity,endVelocity).reversed();
        }
        startVelocity = Math.abs(startVelocity);
        cruiseVelocity = Math.abs(cruiseVelocity);
        endVelocity = Math.abs(endVelocity);
        maxVelocity = Math.abs(maxVelocity);
        maxAcceleration = Math.abs(maxAcceleration);
        maxJerk = Math.abs(maxJerk);

        MotionProfile accelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,maxJerk,0,0, startVelocity,cruiseVelocity);
        MotionProfile decelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,maxJerk,0,0, cruiseVelocity,endVelocity);
        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
        double accelerateAndDecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
        if(accelerateAndDecelerateDistance < PathTotalDistance){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            double cruiseTime = (PathTotalDistance - accelerateAndDecelerateDistance) / cruiseVelocity;
            MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(cruiseSegment);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else if(Math.abs(accelerateAndDecelerateDistance - PathTotalDistance) <= PATH_DISTANCE_ERROR_MARGIN){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else{
            double startEndVMin = Math.min(startVelocity,endVelocity);
            double startEndVMax = Math.max(startVelocity,endVelocity);
            if(cruiseVelocity >= startEndVMin && cruiseVelocity <= startEndVMax){
                return generateMotionProfileWithConstantVelocityAndDistance(cruiseVelocity,PathTotalDistance);
            }else{
                //try to lower / rise cruise speed and see if we can achieve anything better than just cruise at cruise speed.
                final double finalMaxAccel = maxAcceleration, finalMaxJerk = maxJerk;
                final double finalStartVelocity = startVelocity;
                final double finalEndVelocity = endVelocity;
                OrderedValueProvider valueProvider = new OrderedValueProvider() {
                    @Override
                    public boolean orderIncremental() {
                        return true;
                    }

                    @Override
                    public double valueAt(double independentVar) {
                        MotionProfile accelerateProfile = generateMotionProfileFromOneSpeedToAnother(finalMaxAccel,finalMaxJerk,0,0, finalStartVelocity,independentVar);
                        MotionProfile decelerateProfile = generateMotionProfileFromOneSpeedToAnother(finalMaxAccel,finalMaxJerk,0,0, independentVar,finalEndVelocity);
                        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
                        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
                        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
                        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
                        double accelerateAndDecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
                        return accelerateAndDecelerateDistance;
                    }
                };
                double solverMinCruise = cruiseVelocity < startEndVMin ? 0 : startEndVMax;
                double solverMaxCruise = cruiseVelocity < startEndVMin ? startEndVMin : cruiseVelocity;
                double solvedCruiseSpeed = OrderedValueSolver.solve(valueProvider,PATH_DISTANCE_ERROR_MARGIN,solverMinCruise,solverMaxCruise,PathTotalDistance);
                if(solvedCruiseSpeed == OrderedValueSolver.RESULT_NOSOLUTION){
                    return generateMotionProfileWithConstantVelocityAndDistance(cruiseVelocity,PathTotalDistance);
                }else{
                    MotionProfile newAccelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,maxJerk,0,0, startVelocity,solvedCruiseSpeed);
                    MotionProfile newDecelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,maxJerk,0,0, solvedCruiseSpeed,endVelocity);
                    MotionProfile returnProfile = new MotionProfile(startVelocity);
                    returnProfile.addAtEnd(newAccelerateProfile);
                    returnProfile.addAtEnd(newDecelerateProfile);
                    return returnProfile;
                }
            }
        }

         */
    }
    public static MotionProfile generateMotionProfile_JERKUNLIMITED(double maxVelocity, double maxAcceleration, double PathTotalDistance, double startVelocity, double cruiseVelocity, double endVelocity){
        if(PathTotalDistance < 0){
            return generateMotionProfile_JERKUNLIMITED(maxVelocity,maxAcceleration,-PathTotalDistance,startVelocity,cruiseVelocity,endVelocity).negative();
        }
        startVelocity = Math.abs(startVelocity);
        cruiseVelocity = Math.abs(cruiseVelocity);
        endVelocity = Math.abs(endVelocity);
        maxVelocity = Math.abs(maxVelocity);
        maxAcceleration = Math.abs(maxAcceleration);

        MotionProfile accelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,0,0,0, startVelocity,cruiseVelocity);
        MotionProfile decelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,0,0,0, cruiseVelocity,endVelocity);
        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
        double accelerateAndDecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
        if(accelerateAndDecelerateDistance < PathTotalDistance){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            double cruiseTime = (PathTotalDistance - accelerateAndDecelerateDistance) / cruiseVelocity;
            MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(cruiseSegment);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else if(Math.abs(accelerateAndDecelerateDistance - PathTotalDistance) <= PATH_DISTANCE_ERROR_MARGIN){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else {
            double startEndVMin = Math.min(startVelocity, endVelocity);
            double startEndVMax = Math.max(startVelocity, endVelocity);
            if (cruiseVelocity >= startEndVMin && cruiseVelocity <= startEndVMax) {
                return generateMotionProfileWithConstantVelocityAndDistance(cruiseVelocity,PathTotalDistance);
            }else{
                double QuadraticAccel = cruiseVelocity < startEndVMin ? -maxAcceleration : maxAcceleration;
                double QuadraticRight = QuadraticAccel * PathTotalDistance + (Math.pow(startVelocity,2) / 2.0) + (Math.pow(endVelocity,2) / 2.0);
                if(QuadraticRight < 0){
                    return generateMotionProfileWithConstantVelocityAndDistance(cruiseVelocity,PathTotalDistance);
                }
                double newCruiseSpeed = Math.sqrt(QuadraticRight);
                MotionProfile returnProfile = new MotionProfile(startVelocity);
                MotionProfile newStartProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,0,0,0,startVelocity,newCruiseSpeed);
                MotionProfile newEndProfile = generateMotionProfileFromOneSpeedToAnother(maxAcceleration,0,0,0,newCruiseSpeed,endVelocity);
                returnProfile.addAtEnd(newStartProfile);
                returnProfile.addAtEnd(newEndProfile);
                return returnProfile;
            }
        }
    }
    public static MotionProfile generateMotionProfileWithConstantVelocity(double constantVelocity, double duration){
        MotionProfile returnProfile = new MotionProfile(constantVelocity);
        MotionProfileSegment segment = new MotionProfileSegment(0,0,duration);
        returnProfile.addAtEnd(segment);
        return returnProfile;
    }
    public static MotionProfile generateMotionProfileWithConstantVelocityAndDistance(double constantVelocity,double distance){
        double timeRequired = distance / constantVelocity;
        return generateMotionProfileWithConstantVelocity(constantVelocity,timeRequired);
    }
    public static MotionProfile generateMotionProfileFromOneSpeedToAnother(double maxAccel, double maxJerk, double startAcceleration, double endAcceleration, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return generateMotionProfileFromOneSpeedToAnother(maxAccel,maxJerk,startAcceleration,endAcceleration,endVelocity,startVelocity).reversed();
        }
        return __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(maxAccel,startVelocity,endVelocity);
    }
    public static MotionProfile __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(double maxAccel, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(maxAccel,endVelocity,startVelocity).reversed();
        }

        double currentVelocity = startVelocity;

        double Tvelocity = 0;
        Tvelocity = (endVelocity - currentVelocity) /  maxAccel;
        MotionProfileSegment segmentVelocityToEndSpeed = null;
        if(Tvelocity != 0){
            segmentVelocityToEndSpeed = new MotionProfileSegment(maxAccel,0,Tvelocity);
        }
        MotionProfile returnProfile = new MotionProfile(startVelocity);
        if(segmentVelocityToEndSpeed != null){
            returnProfile.addAtEnd(segmentVelocityToEndSpeed);
        }
        return returnProfile;
    }
}
