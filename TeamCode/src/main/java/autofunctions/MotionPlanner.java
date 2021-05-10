package autofunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionPlanner {

    //Rest pow
    public double restPow = 0;
    //Approach rate [0,1] lower means stops farther away
    public double approachRate = 0;
    //Proportional coeff
    public double proportionalCoeff = 0;
    //Target distance
    public double targetDis = 0;
    //Start distance
    public double startDis = 0;
    //Current distance
    public double curDis = 0;
    //Current pow
    public double curPow = 0;
    //Last distance
    public double lastDis = 0;
    //Last time
    public double lastTime = -0.1;
    //Current velocity
    public double curVel = 0;
    //Accuracy
    public double acc = 0;
    //Has the start distance been set
    public boolean hasStartDisBeenSet = false;
    //Timer
    public ElapsedTime timer = new ElapsedTime();

    //Sets the par coefficents
    public void setPAR(double p, double a, double r){
        proportionalCoeff = p;
        approachRate = a;
        restPow = r;
    }
    //Set accuracy
    public void setAcc(double a){
        acc = a;
    }
    //Set the start distance
    public void setStartDis(double sd){
        startDis = sd;
        timer.reset();
        lastTime = -0.1;
        hasStartDisBeenSet = true;
    }
    //Set the target distance
    public void setTargetDis(double td){
        targetDis = td;
    }
    //Calculate velocity vs distance
    public double VofS(){
        return approachRate*Math.pow(Math.abs(targetDis-curDis), 1/approachRate)*Math.signum(targetDis-curDis);
    }
    //Get the rest power required
    public double getRestPow(){
        return Math.signum(targetDis-curDis)*restPow;
    }
    //Update the controller
    public void update(double curPos, double target){
        if(!hasStartDisBeenSet){
            setTargetDis(target);
            setStartDis(curPos);
            hasStartDisBeenSet = true;
        }else {
            setTargetDis(target);
            curDis = (curPos);
            updateValues();
            curPow = (proportionalCoeff * (VofS() - curVel)) + getRestPow();
        }
    }

    //Update the velocity and distance
    public void updateValues(){
        double deltaDis = curDis - lastDis;
        lastDis = curDis;

        double curtime = timer.seconds();
        double deltaTime = curtime - lastTime;
        lastTime = curtime;

        curVel = deltaDis/deltaTime;
    }

    //Get the power for the motor
    public double getPower(){
        return curPow;
    }
    //Has the robot reached the target
    public boolean isDone(){
        return Math.abs(targetDis-curDis) < acc;
    }
    //Reset the controller
    public void reset(){
        hasStartDisBeenSet = false;
    }
}
