package org.darbots.darbotsftclib.libcore.integratedfunctions;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public abstract class FieldOrientedMovementTeleOpControl {
    private float AngleReadingAtHumanPerspective = 0;
    public FieldOrientedMovementTeleOpControl(float HumanPerspectiveReadingCW){
        this.AngleReadingAtHumanPerspective = HumanPerspectiveReadingCW;
    }
    public RobotVector2D getRobotSpeed(RobotVector2D HumanPerspectiveSpeed){
        double[] humanXY = {HumanPerspectiveSpeed.X, HumanPerspectiveSpeed.Y};
        double deltaAngBetweenRobotAndHuman = this.getCurrentAngleCW() - AngleReadingAtHumanPerspective;
        double[] origin = {0,0};
        double[] robotXY = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(humanXY,origin,-deltaAngBetweenRobotAndHuman);
        RobotVector2D returnVector = new RobotVector2D(robotXY[0],robotXY[1],HumanPerspectiveSpeed.getRotationZ());
        return returnVector;
    }

    public abstract float getCurrentAngleCW();
}
