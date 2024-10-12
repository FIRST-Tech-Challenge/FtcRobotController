package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Drivetrain.Utils.Utils;

@Config
public class PoseController {
    public PID xPID;
    public PID yPID;
    public PID tPID;
    public static double kPX = 0.25;
    public static double kPY = 0.3;
    public static double kPTheta= 0.25;
    public static double kIX, kIY, kITheta = 0;
    public static double kDX, kDY, kDTheta = 0;

    public PoseController(){
        xPID = new PID(kPX, kIX, kDX);
        yPID = new PID(kPY, kIY, kDY);
        tPID = new PID(kPTheta, kITheta, kDTheta);
    }
    public SimpleMatrix calculate(SimpleMatrix pose, SimpleMatrix desiredPose){
        SimpleMatrix errorField = new SimpleMatrix(
                new double[][]{
                        new double[]{desiredPose.get(0,0)-pose.get(0,0)},
                        new double[]{desiredPose.get(1,0)-pose.get(1,0)},
                        new double[]{Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0))}
                }
        );

        SimpleMatrix errorRobot = Utils.rotateGlobalToBody(errorField, pose.get(2,0));

        double vX = xPID.calculate(errorRobot.get(0, 0),0);
        double vY = yPID.calculate(errorRobot.get(1, 0),0);
        double omega = tPID.calculate(Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0)),0);

        SimpleMatrix vRobot = new SimpleMatrix (
                new double[][] {
                        new double[]{vX},
                        new double[]{vY},
                        new double[]{omega}
                }
        );
        return Utils.inverseKinematics(vRobot);
    }
}


