package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.MecanumDriveAcceleration;

import org.ejml.simple.SimpleMatrix;
@Config
public class BioBuzzRobotMovementParameters implements RobotMovementParameters {
    public static double robotMass = 10; //kg
    private final double averageMotorForce = 8.5; // N Per Wheel on ground;
    private final double robotTopSpeedStraight = 48; // inches/s
    private final double timeForFullTurn = 1.2; // Seconds
    public BioBuzzRobotMovementParameters(){
    }
    public double getMaxVelocity(double direction, double angleError){
        // 1.5 m/s for the average robot
        return (robotTopSpeedStraight / 2 * Math.sqrt(2)) * MecanumDriveAcceleration.getMovementMagnitude(direction, angleError);
    }
    public double getMaxAngleVelocity(){
        return 360/timeForFullTurn; // degrees per second
    }
    public double getMaxStableAcceleration(double direction, double angleError){
        return getAverageIndividualWheelAcceleration() * MecanumDriveAcceleration.getMovementMagnitude(direction, angleError);
    }
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise){
        return 200; // degrees / s^2 Simple values similar to what road runners defaults are. I will tune a basic chassis to this and make those values the default later.
    }
    private double getAverageIndividualWheelAcceleration(){
        double metersAccel = averageMotorForce / robotMass; // N / Kg = m / s^2
        return metersAccel * (100.0/2.45);
    }

    @Override
    public double getAngleFullPowerToErrorThreshold() {
        return 40;
    }

    public SimpleMatrix getPIDCoefficients(){
        SimpleMatrix coeffs = new SimpleMatrix(3,3);
        coeffs.set(0,0, 1);
        coeffs.set(0,1,0);
        coeffs.set(0,2,0);

        coeffs.set(1,0, 0.2);
        coeffs.set(1,1,0);
        coeffs.set(1,2,0);

        coeffs.set(2,0,1);
        coeffs.set(2,1,0);
        coeffs.set(2,2,0);

        return coeffs;

    }
}
