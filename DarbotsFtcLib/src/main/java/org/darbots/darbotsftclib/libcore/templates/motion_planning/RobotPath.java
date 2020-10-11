package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

public interface RobotPath {
    double getTotalDistance();
    RobotPoint2D getPointAtDistance(double distance);
    DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance);
}
