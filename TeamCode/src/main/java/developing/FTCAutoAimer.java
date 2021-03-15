package developing;


import global.Constants;
import util.Geometry;

public class FTCAutoAimer {
    private final Geometry geometry = new Geometry();
    public SpeedController2 outlController = new SpeedController2();
    public SpeedController2 outrController = new SpeedController2();

    public void update(double robotTheta, double lrDis, double frDis) {
        robotTheta *= Math.PI/180;
        double leftDis = getDisFromCenter(lrDis, robotTheta);
        double frontDis = getDisFromCenter(frDis, robotTheta);
        double s = calcSpeed(frontDis, leftDis);
        outlController.setTargetSpeed(Constants.MAX_OUTTAKE_SPEED/3);
        outrController.setTargetSpeed(Constants.MAX_OUTTAKE_SPEED/3);
    }

    public double getOutlPow(double outlPos) {
        return outlController.getMotorPower(outlPos);
    }

    public double getOutrPow(double outrPos) {
        return outrController.getMotorPower(outrPos);
    }

    public double getDisFromCenter(double len, double robotTheta){
        double d = geometry.lawOfCosinesC(len, Constants.ROBOT_RADIUS, Constants.CENTER_THETA);
        double phi = geometry.lawOfSinesAngle(Constants.ROBOT_RADIUS, d, Constants.CENTER_THETA);
        return d * Math.cos(robotTheta + phi);
    }

    public double calcSpeed(double disFromFront, double disFromLeft) {
        double disToGoal = Math.sqrt(Math.pow(disFromFront, 2) + Math.pow(disFromLeft - Constants.GOAL_FROM_LEFT, 2));
        double deltaHeight = Constants.GOAL_HEIGHT - Constants.SHOOTER_HEIGHT;
        double linearSpeed = disToGoal/Math.cos(Constants.OUTTAKE_ANGLE) * Math.sqrt(4.9/(disToGoal * Math.tan(Constants.OUTTAKE_ANGLE) - deltaHeight));
        return linearSpeed/Constants.SHOOTER_WHEEL_RADIUS;
    }


}