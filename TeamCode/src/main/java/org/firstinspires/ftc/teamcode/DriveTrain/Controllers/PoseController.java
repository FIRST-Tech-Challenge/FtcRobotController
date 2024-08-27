package org.firstinspires.ftc.teamcode.DriveTrain.Controllers;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Utils.Utils;

public class PoseController {
    public PID xPID;
    public PID yPID;
    public PID tPID;
    public static double kPX, kPY, kPTheta = 0;
    public static double kIX, kIY, kITheta = 0;
    public static double kDX, kDY, kDTheta = 0;
    public static double vX;
    public static double vY;
    public static double vTheta;
    public PoseController(){
        xPID = new PID(kPX, kIX, kDX);
        yPID = new PID(kPY, kIY, kDY);
        tPID = new PID(kPTheta, kITheta, kDTheta);
    }
    public SimpleMatrix calculate(SimpleMatrix pose, SimpleMatrix desiredPose){
        vX = xPID.calculate(pose.get(0, 0), desiredPose.get(0, 0));
        vY = yPID.calculate(pose.get(1, 0), desiredPose.get(1, 0));
        vTheta = tPID.calculate(0, Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0)));

        SimpleMatrix vGlobal = new SimpleMatrix (
                new double[][] {
                        new double[]{vX},
                        new double[]{vY},
                        new double[]{vTheta}
                }
        );
        SimpleMatrix nuBody = Utils.rotateGlobalToBody(vGlobal, pose.get(2, 0));
        return Utils.inverseKinematics(nuBody);
    }
}


