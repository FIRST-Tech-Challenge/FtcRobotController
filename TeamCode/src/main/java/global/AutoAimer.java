package global;


import globalfunctions.Constants;
import util.Geometry;

public class AutoAimer {
    //Shot mode 0 is goal, 1,2,3 are powershots
    public int shotMode = 0;
    //Target speed to aim at in rad/s
    public double targetSpeed = 0;
    //Current outtake pos
    public double[] outtakePos = {0,0};
    //Old outtake pos
    public double[] oldOuttakePos = {0,0};
    //Is the robot done shooting
    public boolean isDone = false;
    //Has the robot reached the target pos
    public boolean hasReached = false;
    //Has the target pos been updated
    public boolean hasPosBeenUpdated(){
        return !((outtakePos[0] == oldOuttakePos[0]) && (outtakePos[1] == oldOuttakePos[1]));
    }
    //Sets the outtake pos
    public void setOuttakePos(double[] pos){
        outtakePos = pos;
    }
    //The robot has finished shooting
    public void done(){
        isDone = true;
        hasReached = false;
    }
    //The robot is ready to shoot again
    public void ready(){
        isDone = false;
    }
    //The robot has reached the target pos
    public void reached(){ hasReached = true; }
    //Updates the target speed by calculating the speed using the outtake position
    public void updateTargetSpeed(){
        targetSpeed = calcSpeed((Constants.FIELD_LENGTH - outtakePos[1]/100), outtakePos[0]/100);
    }
    //Resets the outtake pos so that it has been updated
    public void resetOuttakePos(){
        oldOuttakePos = outtakePos;
    }
    //Get the right outtake wheel targget velocity
    public double getOutrTargetVel(){
        return (((considerFriction(targetSpeed+Constants.OUT_SPEED_OFFSET))/Constants.pi2)*Constants.GOBUILDA1_Ticks);
    }
    //Get the left outtake wheel targget velocity
    public double getOutlTargetVel(){
        return (((considerFriction(targetSpeed-Constants.OUT_SPEED_OFFSET))/Constants.pi2)*Constants.GOBUILDA1_Ticks);
    }
    //Go to the next shot mode
    public void nextShotMode(){
        if(shotMode < 3){
            shotMode++;
        }else {
            shotMode = 0;
        }
    }
    //Calculate the speed at which to shoot from
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
        return reverseCalcLinearSpeed(disToGoal, deltaHeight)/Constants.SHOOTER_WHEEL_RADIUS;
    }
    //Calculate the height based on the velocity and the distance
    public double calcHeight(double vel, double disTo){
        double horzVel = vel*Math.cos(Constants.OUTTAKE_ANGLE);
        double vertVe = vel*Math.sin(Constants.OUTTAKE_ANGLE);
        double time = disTo/horzVel;
        return Constants.SHOOTER_HEIGHT + (vertVe*time) - (0.5*9.81*time*time);
    }
    //Calculate the linear speed required given the distance and the target height
    public double reverseCalcLinearSpeed(double disToGoal, double deltaHeight){
        return disToGoal/Math.cos(Constants.OUTTAKE_ANGLE) * Math.sqrt(4.9/(disToGoal * Math.tan(Constants.OUTTAKE_ANGLE) - deltaHeight));
    }
    //The target velocity to the acceleration required
    public double velToAccel(double vel){
        return (vel*vel)/(2*Constants.SHOOT_DIS);
    }
    //The acceleration to the velocity produced
    public double accelToVel(double accel){
        return Math.sqrt(2*accel*Constants.SHOOT_DIS);
    }
    //Consider the effects of friction by adding a friction acceleration constant
    public double considerFriction(double targetSpeed){
        double vtheo = targetSpeed*Constants.SHOOTER_WHEEL_RADIUS;
        double stheo = velToAccel(vtheo);
        double f = Constants.FRICTION_ACCEL;
        double sshouldapply = stheo + f;
        double vshouldapply = accelToVel(sshouldapply);
        return vshouldapply/Constants.SHOOTER_WHEEL_RADIUS;
    }
    //Get the angle the robot should shoot at
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