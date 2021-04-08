package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
    public SpeedController outlController = new SpeedController();
    public SpeedController outrController = new SpeedController();

    public int shotMode = 0;


    public void update(double[] pos) {
        double s = calcSpeed((Constants.FIELD_LENGTH - pos[1]/100), pos[0]/100);
        outlController.setTargetSpeed(s + Constants.OUTTAKE_SPEED_OFFSET);
        outrController.setTargetSpeed(s -  Constants.OUTTAKE_SPEED_OFFSET);
    }

    public double getOutlPow(double outlPos) {
        return outlController.getMotorPower(outlPos);
    }

    public double getOutrPow(double outrPos) {
        return outrController.getMotorPower(outrPos);
    }

    public void nextShotMode(){
        if(shotMode < 3){
            shotMode++;
        }else {
            shotMode = 0;
        }
    }



    public double calcSpeed(double disFromFront, double disFromLeft) {
        double disToGoal = 0;
        double deltaHeight = 0;
        if(shotMode == 0) {
            deltaHeight = Constants.GOAL_HEIGHT - Constants.SHOOTER_HEIGHT;
            disToGoal = Geometry.pythagoreanC(disFromFront, disFromLeft-Constants.GOAL_FROM_LEFT);
        }else{
            deltaHeight = Constants.POWERSHOT_HEIGHT - Constants.SHOOTER_HEIGHT;
            disToGoal = Geometry.pythagoreanC(disFromFront, disFromLeft - Constants.POWERSHOT_FROM_LEFT - (Constants.DIS_BETWEEN_POWERSHOTS*(shotMode-1)));
        }
        double linearSpeed = disToGoal/Math.cos(Constants.OUTTAKE_ANGLE) * Math.sqrt(4.9/(disToGoal * Math.tan(Constants.OUTTAKE_ANGLE) - deltaHeight));
        return linearSpeed/Constants.SHOOTER_WHEEL_RADIUS;
    }

    public void resetOuttake(double outlPos, double outrPos){
        outlController.reset(outlPos);
        outrController.reset(outrPos);
    }

    public double getRobotToGoalAngle(double[] pos) {
        double disFromFront = (Constants.FIELD_LENGTH - pos[1]/100);
        double disFromLeft = pos[0]/100;
        if(shotMode == 0) {
            return Math.atan2(disFromFront, disFromLeft - Constants.GOAL_FROM_LEFT);
        }else{
            return Math.atan2(disFromFront, disFromLeft - Constants.POWERSHOT_FROM_LEFT - (Constants.DIS_BETWEEN_POWERSHOTS*(shotMode-1)));
        }
    }



}