package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID.functionType;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;

@Config //Allows tuning these parameters through FTC Dashboard
public class PoseController {

    //PID Controllers for X, Y, and Theta (heading)
    public PID xPID;
    public PID yPID;
    public PID tPID;

    //PID Constants for tuning via Dashboard
    public static double kPX = 10.5;
    public static double kPY = 10.5;
    public static double kPTheta= 5;

    //Integral and derivative gains for all axes
    public static double kIX, kIY, kITheta = 0;
    public static double kDX = 0;
    public static double kDY = 0;
    public static double kDTheta = 0;

    //Last known valid pose to avoid NaN issues
    private double lastTheta = 0;
    private double lastX;
    private double lastY;

    /**
     * Constructor for the Pose Controller
     * Initializes PID controllers for x, y, and heading with specific gains.
     */
    public PoseController(){
        this.xPID = new PID(kPX, kIX, kDX, functionType.SQRT);
        this.yPID = new PID(kPY, kIY, kDY, functionType.SQRT);
        this.tPID = new PID(kPTheta, kITheta, kDTheta, functionType.SQRT);
    }

    /**
     * Calculates a velocity vector to move from a current pose to a desired pose.
     * Applies PID control in the robot's frame and returns motor power commands.
     *
     * @param pose          The current robot pose [x; y; heading] as a column matrix.
     * @param desiredPose   The target robot pose [x; y; heading] as a column matrix.
     * @return              A 3x1 matrix representing the drive power to apply to each wheel.
     */
    public SimpleMatrix calculate(SimpleMatrix pose, SimpleMatrix desiredPose){
        // if current pose has invalid values, use the last valid pose
        if (pose.hasUncountable()){
            pose.set(0, 0, lastX);
            pose.set(1, 0, lastY);
            pose.set(2,0,lastTheta);
        } else {
            // update last known valid pose
            lastX = pose.get(0, 0);
            lastY = pose.get(1, 0);
            lastTheta = pose.get(2,0);
        }


        // Compute error in the global field fram
        SimpleMatrix errorField = new SimpleMatrix(
                new double[][]{
                        new double[]{desiredPose.get(0,0)-pose.get(0,0)},
                        new double[]{desiredPose.get(1,0)-pose.get(1,0)},
                        new double[]{Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0))}
                }
        );

        // Convert error to the robot's frame of reference
        SimpleMatrix errorRobot = Utils.rotateGlobalToBody(errorField, pose.get(2,0));

        // Use PID controllers to calculate control effect in robot frame
        double vX = xPID.calculate(errorRobot.get(0, 0),0);
        double vY = yPID.calculate(errorRobot.get(1, 0),0);
        double omega = tPID.calculate(Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0)),0);

        // Create a velocity vector in robot frame
        SimpleMatrix vRobot = new SimpleMatrix (
                new double[][] {
                        new double[]{vX},
                        new double[]{vY},
                        new double[]{omega}
                }
        );

        // Convert robot-frame velocity vector to individual wheel powers
        return Utils.inverseKinematics(vRobot);
    }
}


