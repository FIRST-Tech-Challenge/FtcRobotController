package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
    public SpeedController outlController = new SpeedController();
    public SpeedController outrController = new SpeedController();

    public void update(double robotTheta, double[] pos) {
        robotTheta *= Math.PI/180;


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