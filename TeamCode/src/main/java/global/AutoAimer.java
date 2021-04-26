package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
    public int shotMode = 0;

    public double targetSpeed = 0;

    public double[] outtakePos = {0,0};
    public double[] oldOuttakePos = {0,0};

    public boolean isDone = false;
    public boolean hasReached = false;

    public double debug = 0;

    public boolean override = false;

    public boolean hasPosBeenUpdated(){
        return !((outtakePos[0] == oldOuttakePos[0]) && (outtakePos[1] == oldOuttakePos[1]));
    }
    public void setOuttakePos(double[] pos){
        outtakePos = pos;
    }

    public void done(){
        isDone = true;
        hasReached = false;
    }
    public void ready(){
        isDone = false;
    }
    public void reached(){ hasReached = true; }



    public void updateTargetSpeed(){
        // 0.222520388095038
        // 9.25 m/s -> 8.31 m/s ->
        targetSpeed = calcSpeed((Constants.FIELD_LENGTH - outtakePos[1]/100), outtakePos[0]/100);
        oldOuttakePos = outtakePos;
    }

    public double getOutrTargetVel(){
        return (((targetSpeed+Constants.OUTR_SPEED_OFFSET + debug)/Constants.pi2)*Constants.GOBUILDA1_Ticks);
    }
    public double getOutlTargetVel(){
        return (((targetSpeed+Constants.OUTL_SPEED_OFFSET + debug)/Constants.pi2)*Constants.GOBUILDA1_Ticks);
    }

    public void nextShotMode(){
        if(shotMode < 3){
            shotMode++;
        }else {
            shotMode = 0;
        }
    }



    public double calcSpeed(double disFromFront, double disFromLeft) {
        double disToGoal;
        double deltaHeight;
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

    public double getRobotToGoalAngle(double[] pos) {
        double offset = Constants.CURVATURE_TAN_THETA * pos[1]/100;
        double disFromFront = (Constants.FIELD_LENGTH - pos[1]/100);
        double disFromLeft = pos[0]/100;
        if(shotMode == 0) {
            return Math.toDegrees(Constants.halfPi - Math.atan2(disFromFront, disFromLeft - Constants.GOAL_FROM_LEFT - offset));
        }else{
            return Math.toDegrees(Constants.halfPi - Math.atan2(disFromFront, disFromLeft - Constants.POWERSHOT_FROM_LEFT - (Constants.DIS_BETWEEN_POWERSHOTS*(shotMode-1)) - offset));
        }
    }





}