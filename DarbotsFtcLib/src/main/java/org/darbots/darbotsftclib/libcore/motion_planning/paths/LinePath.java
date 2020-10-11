package org.darbots.darbotsftclib.libcore.motion_planning.paths;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class LinePath implements RobotPath {
    public double targetX, targetY;

    public LinePath(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    public LinePath(LinePath oldPath){
        this.targetX = oldPath.targetX;
        this.targetY = oldPath.targetY;
    }
    
    @Override
    public double getTotalDistance() {
        return Math.sqrt(Math.pow(targetX,2) + Math.pow(targetY,2));
    }

    @Override
    public RobotPoint2D getPointAtDistance(double distance) {
        double x=0, y=0;
        x = distance / this.getTotalDistance() * targetX;
        y = distance / this.getTotalDistance() * targetY;
        return new RobotPoint2D(x,y);
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance) {
        return new DarbotsDerivative(this.targetX,this.targetY);
    }
}
