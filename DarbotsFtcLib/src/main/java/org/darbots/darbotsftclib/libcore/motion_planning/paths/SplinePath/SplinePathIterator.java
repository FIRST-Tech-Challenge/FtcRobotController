package org.darbots.darbotsftclib.libcore.motion_planning.paths.SplinePath;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPathIterator;

import java.util.NoSuchElementException;

public class SplinePathIterator implements RobotPathIterator {
    private double m_Resolution;
    private RobotPathIteratorStatus m_CurrentStatus;
    private SplinePath m_Path;
    private double m_TotalDistance;

    public SplinePathIterator(SplinePath path){
        this.m_Path = path;
        this.__setup();
    }

    public SplinePathIterator(SplinePathIterator oldIterator){
        this.m_Resolution = oldIterator.m_Resolution;
        this.m_CurrentStatus = new RobotPathIteratorStatus(oldIterator.m_CurrentStatus);
        this.m_Path = oldIterator.m_Path;
        this.m_TotalDistance = oldIterator.m_TotalDistance;
    }

    private void __setup(){
        RobotPoint2D currentPoint = new RobotPoint2D(0,0);
        this.m_CurrentStatus = new RobotPathIteratorStatus(0,currentPoint);
        this.m_TotalDistance = this.m_Path.getTotalDistance();
        this.m_Resolution = this.m_Path.m_PathIntegrationResolution;
    }

    @Override
    public RobotPathIteratorStatus forward(double deltaDistance) throws NoSuchElementException {
        if(deltaDistance == 0){
            return this.current();
        }
        //Retrieve Info
        double iteratorTargetDistance = this.m_CurrentStatus.currentDistance + deltaDistance;
        boolean isXSpline = this.m_Path.m_SplineType == SplinePath.SplineType.X_BASED_SPLINE;
        boolean isReversedSpline = this.m_Path.m_Spline.getMinX() != 0;
        double xOrigin = 0;
        double xExtreme = isReversedSpline ? this.m_Path.m_Spline.getMinX() : this.m_Path.m_Spline.getMaxX();

        //Convert current status to a start status based on X/Y Spline Type
        RobotPathIteratorStatus iteratorStartStatus =
                this.m_Path.m_SplineType == SplinePath.SplineType.X_BASED_SPLINE ?
                        new RobotPathIteratorStatus(this.m_CurrentStatus) :
                        new RobotPathIteratorStatus(
                                this.m_CurrentStatus.currentDistance,
                                new RobotPoint2D(
                                        this.m_CurrentStatus.currentPoint.Y,
                                        this.m_CurrentStatus.currentPoint.X
                                )
                        );
        double xCounter = iteratorStartStatus.currentPoint.X;
        double yAtCounter = iteratorStartStatus.currentPoint.Y;
        double distanceCounter = iteratorStartStatus.currentDistance;

        //Fallback xCounter, yAtCounter, distanceCounter to a checkpoint, where the xCounter is a multiple of the resolution.
        {
            double distanceBetweenXAndXOrigin = xCounter - xOrigin;
            double xRemainder = distanceBetweenXAndXOrigin % this.m_Resolution;
            if(xRemainder != 0){
                double newX = xCounter - xRemainder;
                double newY = this.m_Path.m_Spline.interpolate(newX);
                distanceCounter -= Math.sqrt(Math.pow(newY - yAtCounter,2) + Math.pow(newX - xCounter,2));
                xCounter = newX;
                yAtCounter = newY;
            }
        }

        if(distanceCounter == iteratorTargetDistance){
            return returnStatus(isXSpline,xCounter,yAtCounter,distanceCounter);
        }

        boolean isForwardDistance = distanceCounter <= iteratorTargetDistance;

        if(isForwardDistance){
            double deltaXEveryTime = isReversedSpline ? -this.m_Resolution : this.m_Resolution;
            double targetX, targetY;
            double deltaX, deltaY, deltaTargetDistance;
            while(xCounter != xExtreme){
                targetX = xCounter + deltaXEveryTime;
                if((isReversedSpline && targetX < xExtreme) || ((!isReversedSpline) && targetX > xExtreme)){
                    targetX = xExtreme;
                }
                targetY = this.m_Path.m_Spline.interpolate(targetX);
                deltaX = targetX - xCounter;
                deltaY = targetY - yAtCounter;
                deltaTargetDistance = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
                if(distanceCounter + deltaTargetDistance >= iteratorTargetDistance) {
                    double distRemaining = iteratorTargetDistance - distanceCounter;
                    double tempFactor = distRemaining / deltaTargetDistance;
                    double newDeltaX = tempFactor * deltaX;
                    double newDeltaY = tempFactor * deltaY;
                    return returnStatus(isXSpline, xCounter + newDeltaX, yAtCounter + newDeltaY, iteratorTargetDistance);
                }
                xCounter = targetX;
                yAtCounter = targetY;
                distanceCounter += deltaTargetDistance;
            }
            throw new NoSuchElementException("Iterated through the whole spline, but did not find the point");
        }else{ //!isForwardDistance
            double deltaXEveryTime = isReversedSpline ? this.m_Resolution : -this.m_Resolution;
            double targetX, targetY;
            double deltaX, deltaY, deltaTargetDistance;
            while(xCounter != xExtreme){
                targetX = xCounter + deltaXEveryTime;
                if((isReversedSpline && targetX > xExtreme) || ((!isReversedSpline) && targetX < xExtreme)){
                    targetX = xExtreme;
                }
                targetY = this.m_Path.m_Spline.interpolate(targetX);
                deltaX = targetX - xCounter;
                deltaY = targetY - yAtCounter;
                deltaTargetDistance = -Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
                if(distanceCounter + deltaTargetDistance <= iteratorTargetDistance) {
                    double distRemaining = iteratorTargetDistance - distanceCounter; //neagtive
                    double tempFactor = distRemaining / deltaTargetDistance;
                    double newDeltaX = tempFactor * deltaX;
                    double newDeltaY = tempFactor * deltaY;
                    return returnStatus(isXSpline, xCounter + newDeltaX, yAtCounter + newDeltaY, iteratorTargetDistance);
                }
                
                xCounter = targetX;
                yAtCounter = targetY;
                distanceCounter += deltaTargetDistance;
            }
            throw new NoSuchElementException("Iterated through the whole spline, but did not find the point");
        }
    }

    private RobotPathIteratorStatus returnStatus(boolean isXSpline, double xCounter, double yCounter, double distanceCounter){
        if(isXSpline){
            this.m_CurrentStatus.currentPoint.X = xCounter;
            this.m_CurrentStatus.currentPoint.Y = yCounter;
        }else{
            this.m_CurrentStatus.currentPoint.X = yCounter;
            this.m_CurrentStatus.currentPoint.Y = xCounter;
        }
        this.m_CurrentStatus.currentDistance = distanceCounter;
        return this.m_CurrentStatus;
    }

    @Override
    public RobotPathIteratorStatus backward(double deltaBackwardDistance) throws NoSuchElementException {
        return forward(-deltaBackwardDistance);
    }

    @Override
    public RobotPathIteratorStatus current() {
        return this.m_CurrentStatus;
    }

    @Override
    public double getCurrentDistance() {
        return this.m_CurrentStatus.currentDistance;
    }

    @Override
    public double getTotalDistance() {
        return this.m_TotalDistance;
    }
}
