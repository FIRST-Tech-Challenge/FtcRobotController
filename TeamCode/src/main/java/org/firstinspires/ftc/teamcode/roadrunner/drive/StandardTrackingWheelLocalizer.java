package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPOVVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer implements Localizer {
    public static double Y_MULTIPLIER = 1/*0.896707*/;
    public static double X_MULTIPLIER = 1/*107.5/119*/;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.3779/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.12*X_MULTIPLIER/*10.9951*X_MULTIPLIER*/; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -5.8*Y_MULTIPLIER/*-4.32939*/; // in; offset of the lateral wheel

    private double[] lastTicks = {0,0,0};

    private double ticks_per_radian = TICKS_PER_REV*LATERAL_DISTANCE/(WHEEL_RADIUS*2*PI), ticks_per_inch = TICKS_PER_REV/(WHEEL_RADIUS*2*PI),
    aOffset=0, lastAngle=0;

    //start 5.1,5.5,...
    //end 4 low
    //robot_wid*pi*8/(1.377*pi)*8192
    //1300000 = lateralDistance * 8192/1.377 * 10 * 2
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "hangerMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorRightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorLeftFront"));
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    public List<Double> getWheelPositions() {
        packet.put("left",leftEncoder.getCurrentPosition());
        packet.put("right",rightEncoder.getCurrentPosition());
        packet.put("back",frontEncoder.getCurrentPosition());
        packet.put("x",getPoseEstimate().getX());
        packet.put("y",getPoseEstimate().getY());
        packet.put("a",getPoseEstimate().getHeading()*180/PI);        /*op.telemetry.update();*/
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()*X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition()*X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCurrentPosition()*Y_MULTIPLIER)
        );
    }

    @NonNull
    public List<Double>  getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()*X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()*X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()*Y_MULTIPLIER)
        );
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
        double xpos = currentPose.getX(), ypos=currentPose.getY(), angle = currentPose.getHeading();
        double[] nowTicks = {leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(),
                frontEncoder.getCurrentPosition()};
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
        if(angle!=lastAngle){
            aOffset += angle-lastAngle;
        }
        angle = aOffset + (nowTicks[1]-nowTicks[0])/ticks_per_radian;
        lastAngle = angle;
        xpos += deltaX;
        ypos += deltaY;
        currentPose = new Pose2d(xpos,ypos,angle);
        double[] velo = {leftEncoder.getCorrectedVelocity()/ticks_per_inch, rightEncoder.getCorrectedVelocity()/ticks_per_inch,
                frontEncoder.getCorrectedVelocity()/ticks_per_inch};
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
        setPoseEstimate(currentPose);
        if(currentPose!=null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }
    }

    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        currentPose=pose2d;
    }

    public Pose2d getPoseVelocity() {
        return currentPOVVelocity;
    }
}
