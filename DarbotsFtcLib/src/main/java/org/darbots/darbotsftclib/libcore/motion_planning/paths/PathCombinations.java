package org.darbots.darbotsftclib.libcore.motion_planning.paths;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

import java.util.ArrayList;

public class PathCombinations implements RobotPath {
    private ArrayList<RobotPath> m_Paths;

    public PathCombinations(){
        this.m_Paths = new ArrayList<RobotPath>();
    }

    public PathCombinations(PathCombinations oldCombination){
        this.m_Paths = new ArrayList<RobotPath>();
        this.m_Paths.addAll(oldCombination.m_Paths);
    }

    public ArrayList<RobotPath> getPathList(){
        return this.m_Paths;
    }

    @Override
    public double getTotalDistance() {
        double distanceCounter = 0;
        for(RobotPath i : this.m_Paths){
            distanceCounter += i.getTotalDistance();
        }
        return distanceCounter;
    }

    @Override
    public RobotPoint2D getPointAtDistance(double distance) {
        if(distance == 0){
            return new RobotPoint2D(0,0);
        }
        double distanceCounter = 0;
        for(RobotPath i : this.m_Paths){
            double currentDistance = i.getTotalDistance();
            if(distanceCounter + currentDistance >= distance){
                return i.getPointAtDistance(distance - distanceCounter);
            }else{
                distanceCounter += currentDistance;
            }
        }
        if(this.m_Paths.isEmpty()){
            return new RobotPoint2D(0,0);
        }else{
            RobotPath lastPath = this.m_Paths.get(this.m_Paths.size() - 1);
            return lastPath.getPointAtDistance(lastPath.getTotalDistance());
        }
    }

    /*
    @Override
    public DarbotsDerivative getDerivativeYOverXAtDistance(double distance) {
        double distanceCounter = 0;
        for(RobotPath i : this.m_Paths){
            double currentDistance = i.getTotalDistance();
            if(distanceCounter + currentDistance >= distance){
                return i.getDerivativeYOverXAtDistance(distance - distanceCounter);
            }else{
                distanceCounter += currentDistance;
            }
        }
        if(this.m_Paths.isEmpty()){
            return null;
        }else{
            RobotPath lastPath = this.m_Paths.get(this.m_Paths.size() - 1);
            return null;
        }
    }
    */

    @Override
    public DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance) {
        DarbotsDerivative derivativeCounter = new DarbotsDerivative(0,0);
        double distanceCounter = 0;
        for(RobotPath i : this.m_Paths){
            double currentDistance = i.getTotalDistance();
            if(distanceCounter + currentDistance > startDistance && distanceCounter < endDistance){
                DarbotsDerivative currentDerivative = null;
                double pieceStartDuration, pieceEndDuration;
                if(distanceCounter <= startDistance){ //first segment
                    pieceStartDuration = startDistance - distanceCounter;
                }else{
                    pieceStartDuration = 0;
                }
                if(distanceCounter + currentDistance <= endDistance){ //first / medium segment
                    pieceEndDuration = currentDistance;
                }else{ //distanceCounter + currentDistance > endDistance (last segment)
                    pieceEndDuration = endDistance - distanceCounter;
                }
                currentDerivative = i.getDerivativeYOverXBetweenDistance(pieceStartDuration,pieceEndDuration);
                derivativeCounter.deltaX += currentDerivative.deltaX;
                derivativeCounter.deltaY += currentDerivative.deltaY;
            }
        }
        return derivativeCounter;
    }
}
