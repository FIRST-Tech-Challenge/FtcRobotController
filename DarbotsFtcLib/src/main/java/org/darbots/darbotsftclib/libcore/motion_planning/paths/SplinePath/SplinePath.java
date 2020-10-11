package org.darbots.darbotsftclib.libcore.motion_planning.paths.SplinePath;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.SplineInterpolator;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPathIterator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SplinePath implements RobotPath {
    protected static enum SplineType{
        X_BASED_SPLINE,
        Y_BASED_SPLINE;
    }
    protected double m_PathIntegrationResolution;
    protected SplineType m_SplineType;
    protected SplineInterpolator m_Spline;
    private List<RobotPoint2D> m_InterpolationPoints;
    private SplinePathIterator m_InternalIterator;

    public static List<RobotPoint2D> sortPointsBasedOnX(List<RobotPoint2D> points){
        List<RobotPoint2D> sortedList = new ArrayList<>(points);
        Collections.sort(sortedList,new RobotPoint2D.XComparator());
        return sortedList;
    }

    public static List<RobotPoint2D> sortPointsBasedOnY(List<RobotPoint2D> points){
        List<RobotPoint2D> sortedList = new ArrayList<>(points);
        Collections.sort(sortedList,new RobotPoint2D.YComparator());
        return sortedList;
    }

    public static boolean recurringXInPoints(List<RobotPoint2D> sortedPoints){
        if(sortedPoints.isEmpty()){
            return false;
        }
        double lastX = 0;
        lastX = sortedPoints.get(0).X;
        double currentX;
        for(int i = 1; i < sortedPoints.size(); i++){
            currentX = sortedPoints.get(i).X;
            if(currentX == lastX){
                return true;
            }
            lastX = currentX;
        }
        return false;
    }

    public static boolean recurringYInPoints(List<RobotPoint2D> sortedPoints){
        if(sortedPoints.isEmpty()){
            return false;
        }
        double lastY = 0;
        lastY = sortedPoints.get(0).Y;
        double currentY;
        for(int i = 1; i < sortedPoints.size(); i++){
            currentY = sortedPoints.get(i).Y;
            if(currentY == lastY){
                return true;
            }
            lastY = currentY;
        }
        return false;
    }

    public SplinePath(List<RobotPoint2D> points, double integrationResolution) throws IllegalArgumentException{
        this.m_InterpolationPoints = points;

        List<RobotPoint2D> realPoints = new ArrayList<RobotPoint2D>(points);
        RobotPoint2D origin = new RobotPoint2D(0,0);
        realPoints.add(origin);
        List<RobotPoint2D> sortedXPoints = sortPointsBasedOnX(realPoints);
        List<RobotPoint2D> sortedYPoints = sortPointsBasedOnY(realPoints);
        if((sortedXPoints.get(0).equals(origin) || sortedXPoints.get(sortedXPoints.size() - 1).equals(origin)) && (!recurringXInPoints(sortedXPoints))){
            List<Double> x = new ArrayList<>();
            List<Double> y = new ArrayList<>();
            for(RobotPoint2D i : sortedXPoints){
                x.add(i.X);
                y.add(i.Y);
            }
            this.m_Spline = SplineInterpolator.createMonotoneCubicSpline(x,y);
            this.m_SplineType = SplineType.X_BASED_SPLINE;
        }else if((sortedYPoints.get(0).equals(origin) || sortedYPoints.get(sortedYPoints.size() - 1).equals(origin)) && (!recurringYInPoints(sortedYPoints))){
            List<Double> x = new ArrayList<>();
            List<Double> y = new ArrayList<>();
            for(RobotPoint2D i : sortedYPoints){
                x.add(i.Y);
                y.add(i.X);
            }
            this.m_Spline = SplineInterpolator.createMonotoneCubicSpline(x,y);
            this.m_SplineType = SplineType.Y_BASED_SPLINE;
        }else{
            throw new IllegalArgumentException("The origin does not appear on both ends of the interpolated spline");
        }
        this.m_PathIntegrationResolution = Math.abs(integrationResolution);
        this.m_InternalIterator = new SplinePathIterator(this);
    }

    public SplinePath(SplinePath oldPath){
        this.m_PathIntegrationResolution = oldPath.m_PathIntegrationResolution;
        this.m_SplineType = oldPath.m_SplineType;
        this.m_Spline = new SplineInterpolator(oldPath.m_Spline);
        this.m_InterpolationPoints = oldPath.m_InterpolationPoints;
        this.m_InternalIterator = new SplinePathIterator(this);
    }

    public SplinePath(SplinePath oldPath, double newResolution){
        this.m_PathIntegrationResolution = Math.abs(newResolution);
        this.m_SplineType = oldPath.m_SplineType;
        this.m_Spline = new SplineInterpolator(oldPath.m_Spline);
        this.m_InterpolationPoints = oldPath.m_InterpolationPoints;
        this.m_InternalIterator = new SplinePathIterator(this);
    }

    public double getIntegrationResolution(){
        return this.m_PathIntegrationResolution;
    }

    public List<RobotPoint2D> getInterpolationPoints(){
        return this.m_InterpolationPoints;
    }

    @Override
    public double getTotalDistance() {
        double smallestX = this.m_Spline.getMinX();
        double biggestX = this.m_Spline.getMaxX();

        double xCounter = 0;
        double yAtCounter = 0;

        double distCounter = 0;

        double deltaXEveryTime = 0;

        boolean reversedSpline;
        if(smallestX == 0){
            deltaXEveryTime = this.m_PathIntegrationResolution;
            reversedSpline = false;
        }else{ //biggestX == 0
            deltaXEveryTime = -this.m_PathIntegrationResolution;
            reversedSpline = true;
        }

        double xTarget;
        double yTarget;
        if(!reversedSpline) {
            while (xCounter < biggestX) {
                xTarget = xCounter + deltaXEveryTime;
                if (xTarget > biggestX) {
                    xTarget = biggestX;
                }
                yTarget = this.m_Spline.interpolate(xTarget);
                double distBetween = Math.sqrt(Math.pow(yTarget - yAtCounter, 2) + Math.pow(xTarget - xCounter, 2));
                distCounter += distBetween;
                xCounter = xTarget;
                yAtCounter = yTarget;
            }
        }else{
            while (xCounter > smallestX) {
                xTarget = xCounter + deltaXEveryTime;
                if (xTarget < smallestX) {
                    xTarget = smallestX;
                }
                yTarget = this.m_Spline.interpolate(xTarget);
                double distBetween = Math.sqrt(Math.pow(yTarget - yAtCounter, 2) + Math.pow(xTarget - xCounter, 2));
                distCounter += distBetween;
                xCounter = xTarget;
                yAtCounter = yTarget;
            }
        }
        return distCounter;
    }

    @Override
    public RobotPoint2D getPointAtDistance(double distance) {
        double distanceToGo = distance - m_InternalIterator.getCurrentDistance();
        RobotPathIterator.RobotPathIteratorStatus status = m_InternalIterator.forward(distanceToGo);
        RobotPoint2D pointAtDistance = new RobotPoint2D(status.currentPoint);
        return pointAtDistance;
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance) {
        RobotPoint2D startPoint = this.getPointAtDistance(startDistance);
        RobotPoint2D endPoint = this.getPointAtDistance(endDistance);
        return new DarbotsDerivative(endPoint.X - startPoint.X,endPoint.Y - startPoint.Y);
    }
}
