package org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPOVVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;


public class OdometryTracker extends Tracker {
    private DcMotorEx encoderLeft, encoderRight, encoderBack;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.3779/2;

    public static double LATERAL_DISTANCE = 10.9;
    public static double FORWARD_OFFSET = -4.35;
    private double ticks_per_inch, ticks_per_radian;
    private double[] lastTicks = {0, 0, 0}, odomconst = {1,1,1};
    private boolean high = false;
    public OdometryTracker() {
        super();
        encoderLeft = (DcMotorEx) op.hardwareMap.dcMotor.get("leftEncoder");
        encoderRight = (DcMotorEx) op.hardwareMap.dcMotor.get("rightEncoder");
        encoderBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        encoderBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticks_per_inch = TICKS_PER_REV/(2*PI*WHEEL_RADIUS);
        ticks_per_radian = ticks_per_inch * LATERAL_DISTANCE;
    }
    public double[][] multiplyMatrix(int row1, int col1, double A[][], int row2, int col2, double B[][]) {
        int i, j, k;

        if (row2 != col1) {
            return null;
        }

        double C[][] = new double[row1][col2];

        // Multiply the two matrices
        for (i = 0; i < row1; i++) {
            for (j = 0; j < col2; j++) {
                for (k = 0; k < row2; k++)
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        return C;
    }
    public void update() {
        double[] nowTicks = {odomconst[0]*encoderLeft.getCurrentPosition(), odomconst[1]*encoderRight.getCurrentPosition(),
                odomconst[2]*encoderBack.getCurrentPosition()};
        double[] deltaTicks = {nowTicks[0] - lastTicks[0], nowTicks[1] - lastTicks[1], nowTicks[2] - lastTicks[2]};
        lastTicks = nowTicks;
        double deltaAngle = (deltaTicks[1] - deltaTicks[0]) / ticks_per_radian;
        if(deltaAngle==0){
            deltaAngle=0.00000000001;
        }
        double [][] initalAngle = {{cos(angle),-sin(angle),0},
                {sin(angle),cos(angle),0},
                {0,0,1}};
        double [][] deltaMatrix = {{sin(deltaAngle)/deltaAngle,(cos(deltaAngle)-1)/deltaAngle,0},
                {(1-cos(deltaAngle))/deltaAngle,sin(deltaAngle)/deltaAngle,0},
                {0,0,1}};
        double [][] robotDelta = {{(deltaTicks[0]+deltaTicks[1])*0.5/ticks_per_inch},
                {deltaTicks[2]/ticks_per_inch - (FORWARD_OFFSET*deltaAngle)},
                {deltaAngle}};
        double[][] partialSolve = multiplyMatrix(3,3, initalAngle, 3,3, deltaMatrix);

        double [][] finalSolve = multiplyMatrix(3,3, partialSolve, 3,1, robotDelta);

        double deltaX = finalSolve[0][0];
        double deltaY = finalSolve[1][0];
        angle = (nowTicks[1]-nowTicks[0])/ticks_per_radian;
        xpos += deltaX;
        ypos += deltaY;
        currentPose = new Pose2d(xpos,ypos,angle);
        double[] velo = {odomconst[0]*encoderLeft.getVelocity()/ticks_per_inch, odomconst[1]*encoderRight.getVelocity()/ticks_per_inch,
                odomconst[2]*encoderBack.getVelocity()/ticks_per_inch};
        double headingVelo = (velo[0]-velo[1])/LATERAL_DISTANCE;
        currentPOVVelocity = new Pose2d((velo[0]+velo[1])*0.5, velo[2]+headingVelo*LATERAL_DISTANCE, -headingVelo);
        currentVelocity = new Pose2d(currentPOVVelocity.vec().rotated(angle), currentPOVVelocity.getHeading());
        Canvas fieldOverlay = packet.fieldOverlay();
        packet.put("currentPose", currentPose);
        packet.put("currentVelocity", currentVelocity.getX());
        packet.put("currentPOVVelocity", currentPOVVelocity);
        packet.put("leftTicks", nowTicks[0]);
        packet.put("rightTicks", nowTicks[1]);
        packet.put("backTicks", nowTicks[2]);

        if(currentPose!=null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }
    }
    public void resetAngle(){
        while(angle<0){
            angle+=toRadians(360);
        }
        while(angle>toRadians(360)){
            angle-=toRadians(360);
        }
    }
}
