package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
    private final Geometry geometry = new Geometry();
    public SpeedController outlController = new SpeedController();
    public SpeedController outrController = new SpeedController();

    public void update(double robotTheta, double lrDis, double brDis) {
        robotTheta *= Math.PI/180;
        double leftDis = getDisFromCenter(lrDis, robotTheta);
        double frontDis = Constants.FIELD_LENGTH - getDisFromCenter(brDis, robotTheta);


//        double s = calcSpeed(frontDis, leftDis);

        double s = calcSpeed(2.07, 1);
        outlController.setTargetSpeed(s);
        outrController.setTargetSpeed(s);
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

    public void resetOuttake(double outlPos, double outrPos){
        outlController.reset(outlPos);
        outrController.reset(outrPos);
    }


}