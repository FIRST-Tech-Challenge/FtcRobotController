package org.firstinspires.ftc.team12397.visionSystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Class to return compressed, organized, and easily transformable data for LLtdc
 * @see LLtdc
 */
public class TdcReturnObject {
    private final AngleUnit angleUnit = AngleUnit.RADIANS; // radians
    private final DistanceUnit distanceUnit = DistanceUnit.INCH; // inches
    private double yawCorrection;
    private double robotXCorrection;
    private double robotYCorrection;
    private double armYCorrection;
    private double clawYawCorrection;
    private List<List<Double>> cornerList;

    /**
     * pass in radians and inches.
     * @param yC
     * @param rXc
     * @param rYc
     * @param aYc
     * @param cYc
     */
    public TdcReturnObject(double yC, double rXc, double rYc, double aYc, double cYc, List<List<Double>> cornerL){
        yawCorrection = yC;
        robotXCorrection = rXc; robotYCorrection = rYc;
        armYCorrection = aYc;
        clawYawCorrection = cYc;
        cornerList = cornerL;
    }


    public double getYawCorrection(AngleUnit angleUnit){
        return angleUnit.fromUnit(this.angleUnit, yawCorrection);
    }

    public double getRobotXCorrection(DistanceUnit distanceUnit){
        return distanceUnit.fromUnit(this.distanceUnit, robotXCorrection);
    }

    public double getRobotYCorrection(DistanceUnit distanceUnit){
        return distanceUnit.fromUnit(this.distanceUnit, robotYCorrection);
    }

    public double getArmYCorrection(DistanceUnit distanceUnit){
        return distanceUnit.fromUnit(this.distanceUnit, armYCorrection);
    }

    public double getClawYawCorrection(AngleUnit angleUnit){
        return angleUnit.fromUnit(this.angleUnit, clawYawCorrection);
    }

    public List<List<Double>> getCornerList(){
        return cornerList;
    }

    /**
     *
     * @return a array of Double objects of the calculated corrections in this order:
     *  yawCorrection, robotXCorrection, robotYCorrection, armYCorrection, clawYawCorrection
     *  Yaws are radians, robot X & Y and armY are in inches.
     */
    public Double[] getObjectArray(){
        return new Double[]{yawCorrection, robotXCorrection, robotYCorrection, armYCorrection, clawYawCorrection};
    }

    public String toString(){
        return "Move the robot " + robotXCorrection + " inches right/left, \n" + robotYCorrection + " inches forward/backward, \n or "
                + yawCorrection + " radians cw/ccw, \n Move the arm " + armYCorrection + " inches forward/backward, \n and rotate the claw " + clawYawCorrection + " radians cw/ccw";
    }


}
