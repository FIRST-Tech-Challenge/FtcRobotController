package org.firstinspires.ftc.teamcode.robots.conceptTrikeBot.utils;

public class TrikeChassisMovementCalc {

    private double startingAngle = 0.0;
    private double inputForwardDist = 0.0;
    private double inputRotateAmount = 0.0;
    private double currentLengthOfBot = 0.0;
    private double requestedLengthOfBot = 0.0;


    public TrikeChassisMovementCalc(double startingAngle, double inputForwardDist, double inputRotateAmount, double currentLengthOfBot, double requestedLengthOfBot){
        this.startingAngle = toRad(startingAngle);
        this.inputForwardDist = inputForwardDist;
        this.inputRotateAmount = inputRotateAmount;
        this.currentLengthOfBot = -abs(currentLengthOfBot); //this needs to be negative to be compatible with the calculations
        this.requestedLengthOfBot = -abs(requestedLengthOfBot); //this needs to be negative to be compatible with the calculations
        redoCalcs();
    }

    public void updateVals(double startingAngle, double inputForwardDist, double inputRotateAmount, double currentLengthOfBot, double requestedLengthOfBot){
        this.startingAngle = toRad(startingAngle);
        this.inputForwardDist = inputForwardDist;
        this.inputRotateAmount = inputRotateAmount;
        this.currentLengthOfBot = -abs(currentLengthOfBot); //this needs to be negative to be compatible with the calculations
        this.requestedLengthOfBot = -abs(requestedLengthOfBot); //this needs to be negative to be compatible with the calculations
        redoCalcs();
    }

    double calcdLeftDist = 0; //this is func N on the desmos model
    double calcdRightDist = 0; //this is func O on the desmos model
    double calcdSwerveDist = 0; //this is func P on the desmos model
    double calcdSwerveAngle = 0; //this is func U on the desmos model

    double[] newLeftWheelPoint = {0.0, 0.0}; // this was w
    double[] newRightWheelPoint = {0.0, 0.0}; // this was z
    double[] newSwerveWheelPoint = {0.0, 0.0}; //this was t
    double[] oldSwerveWheelPoint = {0.0, 0.0}; //this was c
    double requestedAngle = 0; //this was f
    double r90 = toRad(90);


    private void redoCalcs(){
        requestedAngle = inputRotateAmount + r90; //input rotate ammount should be in Rad

        newLeftWheelPoint[0] = Constants.radiusOfDiff * cos(requestedAngle + r90) + inputForwardDist * cos(requestedAngle);
        newLeftWheelPoint[1] = Constants.radiusOfDiff * sin(requestedAngle + r90) + inputForwardDist * sin(requestedAngle);

        newRightWheelPoint[0] = -Constants.radiusOfDiff * cos(requestedAngle + r90) + inputForwardDist * cos(requestedAngle);
        newRightWheelPoint[1] = -Constants.radiusOfDiff * sin(requestedAngle + r90) + inputForwardDist * sin(requestedAngle);

        newSwerveWheelPoint[0] = .5 * (newRightWheelPoint[0] + newLeftWheelPoint[0]) + requestedLengthOfBot * cos(requestedAngle);
        newSwerveWheelPoint[1] = .5 * (newRightWheelPoint[1] + newLeftWheelPoint[1]) + requestedLengthOfBot * sin(requestedAngle);

        oldSwerveWheelPoint[0] = 0.0;
        oldSwerveWheelPoint[1] = currentLengthOfBot;

        calcdLeftDist = sqrt(sqr(newLeftWheelPoint[0] + Constants.radiusOfDiff) + sqr(newLeftWheelPoint[1]));
        calcdRightDist = sqrt(sqr(newRightWheelPoint[0] - Constants.radiusOfDiff) + sqr(newRightWheelPoint[1]));
        calcdSwerveDist = sqrt(sqr(newSwerveWheelPoint[0] - oldSwerveWheelPoint[0]) + sqr(newSwerveWheelPoint[1] - oldSwerveWheelPoint[1]));
        calcdSwerveAngle = asin(newSwerveWheelPoint[0] / calcdSwerveDist);
    }

    public double[] getCalcdDistance(){double[] dists = {calcdLeftDist, calcdRightDist, calcdSwerveDist};return dists;}



    public double[] getRelMotorVels(double loopTime){ //todo- move to under mixer
        double[] vels = {0.0, 0.0, 0.0};

        double tempCalcdLeftVel = (calcdLeftDist / (loopTime / 1E9) / Constants.wheelRadius);
        double tempCalcdRightVel = (calcdRightDist / (loopTime / 1E9) / Constants.wheelRadius);
        double tempCalcdMidVel = (calcdSwerveDist / (loopTime / 1E9) / Constants.wheelRadius);

        //normalize values back down so everything fits the max speeds

        if(tempCalcdLeftVel > Constants.maxRotaryVelOfDiffWheels || tempCalcdRightVel > Constants.maxRotaryVelOfDiffWheels){
            tempCalcdLeftVel /= Constants.maxRotaryVelOfDiffWheels;
            tempCalcdRightVel /= Constants.maxRotaryVelOfDiffWheels;
            tempCalcdMidVel /= Constants.maxRotaryVelOfDiffWheels;
        }
        else if(tempCalcdMidVel > Constants.maxRotaryVelOfBackWheel){
            tempCalcdLeftVel /= Constants.maxRotaryVelOfBackWheel;
            tempCalcdRightVel /= Constants.maxRotaryVelOfBackWheel;
            tempCalcdMidVel /= Constants.maxRotaryVelOfBackWheel;
        }

        vels[0] = tempCalcdLeftVel;
        vels[1] = tempCalcdRightVel;
        vels[2] = tempCalcdMidVel;

        return vels;
    }

    public double getNewSwerveAngle(){
        return calcdSwerveAngle;
    }

    public double toRad(double d){return Math.toRadians(d);}
    private double cos(double a){return Math.cos(a);} //to simplify calc lines
    private double sin(double a){return Math.sin(a);} //to simplify calc lines
    private double sqrt(double a){return Math.sqrt(a);}
    private double sqr(double a){return Math.pow(a,2);}
    private double asin(double a){return Math.asin(a);}
    private double abs(double a){return  Math.abs(a);}
}
