package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

public class ArmKinematics {
    protected Constants constants = new Constants();

    //current positions
    protected double currentBase = 0;
    protected double currentTop = 0;

    //arm angles
    protected double theta; //angle between first stage and ground
    protected double psi; //angle between top stage and first stage
    protected double phi; //angle between claw and top stage

    public ArmKinematics(){

    }

    public double getTheta(double clicks){
        return clicks * constants.DEGS_PER_BASE_CLICK;
    }

    public double getPsi(double clicks){
        return clicks * constants.DEGS_PER_TOP_CLICK;
    }

    public double findHeightToGround(double theta, double psi){
        double distance = 0; //distance from claw to ground
        double h1 = constants.ARM_BASE_RADIUS * Math.sin(theta);
        double h2 = constants.ARM_TOP_RADIUS * Math.sin(theta + psi - 90);

        distance = h1 + h2;
        return distance;
    }

    public void setCurrentPosition(){
        currentBase = 10;
        currentTop = 10;
    }
}
