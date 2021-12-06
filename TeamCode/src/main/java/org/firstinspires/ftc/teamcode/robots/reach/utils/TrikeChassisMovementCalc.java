package org.firstinspires.ftc.teamcode.robots.reach.utils;

public class TrikeChassisMovementCalc {

    private double t = 0.0; //this was t startingAngle
    private double q = 0.0; //this was q inputForwardDist
    private double n = 0.0; // this was n inputRotateAmount
    private double E = 0.0; // E currentLengthOfBot
    private double M = 0.0; //M requestedLengthOfBot


    public TrikeChassisMovementCalc(double startingAngle, double inputForwardDist, double inputRotateAmount, double currentLengthOfBot, double requestedLengthOfBot){
        this.t = toRad(startingAngle);
        this.q = inputForwardDist;
        this.n = inputRotateAmount;
        this.E = -abs(currentLengthOfBot); //this needs to be negative to be compatible with the calculations
        this.M = -abs(requestedLengthOfBot); //this needs to be negative to be compatible with the calculations
        redoCalcs();
    }

    public void updateVals(double startingAngle, double inputForwardDist, double inputRotateAmount, double currentLengthOfBot, double requestedLengthOfBot){
        this.t = toRad(startingAngle);
        this.q = inputForwardDist;
        this.n = inputRotateAmount;
        this.E = -abs(currentLengthOfBot); //this needs to be negative to be compatible with the calculations
        this.M = -abs(requestedLengthOfBot); //this needs to be negative to be compatible with the calculations
        redoCalcs();
    }

    double N = 0; //this is func N on the desmos model calcdLeftDist
    double O = 0; //this is func O on the desmos model calcdRightDist
    double P = 0; //this is func P on the desmos model calcdSwerveDist
    double U = 0; //this is func U on the desmos model calcdSwerveAngle

    public double[] W = {0.0, 0.0}; // this was w newLeftWheelPoint
    public double[] Z = {0.0, 0.0}; // this was z newRightWheelPoint
    public double[] T = {0.0, 0.0}; //this was t newSwerveWheelPoint
    public double[] A = {0.0, 0.0}; //this was A oldLeftWheelPoint
    public double[] B = {0.0, 0.0}; //this was B oldRightWheelPoint
    public double[] C = {0.0, 0.0}; //this was c oldSwerveWheelPoint
    public double f = 0; //this was f
    public double c = Constants.radiusOfDiff;
    double r90 = toRad(90);


    private void redoCalcs(){
        f = n + t + r90; //input rotate ammount should be in Rad

        W[0] = c * cos(f + r90) + q * cos(f);
        W[1] = c * sin(f + r90) + q * sin(f);

        Z[0] = -c * cos(f + r90) + q * cos(f);
        Z[1] = -c * sin(f + r90) + q * sin(f);

        T[0] = .5 * (Z[0] + W[0]) + M * cos(f);
        T[1] = .5 * (Z[1] + W[1]) + M * sin(f);

        A[0] = -c * cos(t);
        A[1] = -c * sin(t);

        B[0] = c * cos(t);
        B[1] = c * sin(t);

        C[0] = E * cos(t + r90);
        C[1] = E * sin(t + r90);

        N = sqrt(sqr(W[0] - A[0]) + sqr(W[1] - A[1]));
        O = sqrt(sqr(Z[0] - B[0]) + sqr(Z[1] - A[1]));
        P = sqrt(sqr(T[0] - C[0]) + sqr(T[1] - C[1]));
        U = asin(T[0] / P);
    }

    public double[] getCalcdDistance(){double[] dists = {N, O, P};return dists;}



    public double[] getRelMotorVels(double loopTime){ //todo- move to under mixer
        double[] vels = {0.0, 0.0, 0.0};

        double tempCalcdLeftVel = (N / (loopTime / 1E9) / Constants.wheelRadius);
        double tempCalcdRightVel = (O / (loopTime / 1E9) / Constants.wheelRadius);
        double tempCalcdMidVel = (P / (loopTime / 1E9) / Constants.wheelRadius);

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
        return U;
    }

    public double toRad(double d){return Math.toRadians(d);}
    private double cos(double a){return Math.cos(a);} //to simplify calc lines
    private double sin(double a){return Math.sin(a);} //to simplify calc lines
    private double sqrt(double a){return Math.sqrt(a);}
    private double sqr(double a){return Math.pow(a,2);}
    private double asin(double a){return Math.asin(a);}
    private double abs(double a){return  Math.abs(a);}
}
