package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
//    public SpeedController outlController = new SpeedController();
//    public SpeedController outrController = new SpeedController();

    public int shotMode = 0;

    public double targetSpeed = 0;

    public double[] outtakePos = {0,0};
    public double[] oldOuttakePos = {0,0};

    public boolean isDone = false;
    public boolean hasReached = false;

    public boolean hasPosBeenUpdated(){
        return (outtakePos[0] == oldOuttakePos[0]) && (outtakePos[1] == oldOuttakePos[1]);
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
        targetSpeed = calcSpeed((Constants.FIELD_LENGTH - outtakePos[1]/100), outtakePos[0]/100);
//        outlController.setTargetSpeed(targetSpeed);
//        outrController.setTargetSpeed(targetSpeed);
        oldOuttakePos = outtakePos;
    }

    public int getOutrTargetVel(){
        return (int) (((targetSpeed/Constants.pi2)+Constants.OUTR_SPEED_OFFSET)*Constants.GOBUILDA1_Ticks);
    }
    public int getOutlTargetVel(){
        return (int) (((targetSpeed/Constants.pi2)+Constants.OUTL_SPEED_OFFSET)*Constants.GOBUILDA1_Ticks);
    }

//
//    public double getOutlPow(double outlPos) {
//        return outlController.getMotorPower(outlPos);
//    }
//
//    public double getOutrPow(double outrPos) {
//        return outrController.getMotorPower(outrPos);
//    }
//
//    public double getOutlTargetPow() {
//        return outlController.targetSpeed/Constants.MAX_OUTTAKE_SPEED;
//    }
//
//    public double getOutrTargetPow() {
//        return outrController.targetSpeed/Constants.MAX_OUTTAKE_SPEED;
//    }

    public void nextShotMode(){
        if(shotMode < 3){
            shotMode++;
        }else {
            shotMode = 0;
        }
    }



    public double calcSpeed(double disFromFront, double disFromLeft) { // 2.9476, 0.87
        double disToGoal;
        double deltaHeight;
        if(shotMode == 0) {
            deltaHeight = Constants.GOAL_HEIGHT - Constants.SHOOTER_HEIGHT; // 0.65
            disToGoal = Geometry.pythagoreanC(disFromFront, disFromLeft-Constants.GOAL_FROM_LEFT); // 2.9476, -0.03 -> 2.94775
        }else{
            deltaHeight = Constants.POWERSHOT_HEIGHT - Constants.SHOOTER_HEIGHT;
            disToGoal = Geometry.pythagoreanC(disFromFront, disFromLeft - Constants.POWERSHOT_FROM_LEFT - (Constants.DIS_BETWEEN_POWERSHOTS*(shotMode-1)));
        }
        // outtake angle = 0.349; cos angle = 0.9397; tan angle = 0.36389
        // disToGoal = 2.94775
        // first part = 3.1369
        // second part = 3.4048
        // linear speed = 10.6806
        double linearSpeed = disToGoal/Math.cos(Constants.OUTTAKE_ANGLE) * Math.sqrt(4.9/(disToGoal * Math.tan(Constants.OUTTAKE_ANGLE) - deltaHeight));
        // shooter wheel radius = 0.05
        // returns 213.6
        return linearSpeed/Constants.SHOOTER_WHEEL_RADIUS;
    }
//
//    public void resetOuttake(double outlPos, double outrPos){
//        outlController.reset(outlPos);
//        outrController.reset(outrPos);
//    }

    public double getRobotToGoalAngle(double[] pos) {
        double disFromFront = (Constants.FIELD_LENGTH - pos[1]/100);
        double disFromLeft = pos[0]/100;
        if(shotMode == 0) {
            return Math.toDegrees(Constants.halfPi - Math.atan2(disFromFront, disFromLeft - Constants.GOAL_FROM_LEFT));
        }else{
            return Math.toDegrees(Constants.halfPi - Math.atan2(disFromFront, disFromLeft - Constants.POWERSHOT_FROM_LEFT - (Constants.DIS_BETWEEN_POWERSHOTS*(shotMode-1))));
        }
    }





}