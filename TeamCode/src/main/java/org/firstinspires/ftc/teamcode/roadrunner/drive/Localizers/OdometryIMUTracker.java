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

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

//5900,4900
//5700,4300
//3853
//2/1.337/pi*8092
public class OdometryIMUTracker extends Tracker {
    private DcMotorEx encoderPar, encoderBack;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.3779 / 2;
    public static double LATERAL_OFFSET = 5.45;
    public static double FORWARD_OFFSET = -4.35;
    private double ticks_per_inch, deltaAngle;
    private double[] lastTicks = {0, 0}, odomconst = {1, -1};
    private final BNO055IMU imu;
    private Orientation lastAngles = null;
    public static float globalAngle = 0;
    private AngularVelocity angularVelocity;

    public OdometryIMUTracker(boolean right) {
        super();

        if (right) {
            encoderPar = (DcMotorEx) op.hardwareMap.dcMotor.get("rightEncoder");
        } else {
            encoderPar = (DcMotorEx) op.hardwareMap.dcMotor.get("leftEncoder");
        }
        encoderPar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        encoderBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        globalAngle = 0;
        deltaAngle = 0;
        lastAngles = null;
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        lastAngles = new Orientation();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ticks_per_inch = TICKS_PER_REV / (2 * PI * WHEEL_RADIUS);

        // make sure the imu gyro is calibrated before continuing.
        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.sleep(50);
            op.idle();
        }
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
        double[] nowTicks = {odomconst[0] * encoderPar.getCurrentPosition(), odomconst[1] * encoderBack.getCurrentPosition()};
        double[] deltaTicks = {nowTicks[0] - lastTicks[0], nowTicks[1] - lastTicks[1]};
        lastTicks = nowTicks;
        double newAngle = getAngle();
        double[][] initalAngle = {{cos(angle), -sin(angle), 0},
                {sin(angle), cos(angle), 0},
                {0, 0, 1}};
        double[][] deltaMatrix = {{sin(deltaAngle) / deltaAngle, (cos(deltaAngle) - 1) / deltaAngle, 0},
                {(1 - cos(deltaAngle)) / deltaAngle, sin(deltaAngle) / deltaAngle, 0},
                {0, 0, 1}};
        double[][] robotDelta = {{(deltaTicks[0]) / ticks_per_inch},
                {deltaTicks[1] / ticks_per_inch - (FORWARD_OFFSET * deltaAngle)},
                {deltaAngle}};
        double[][] partialSolve = multiplyMatrix(3, 3, initalAngle, 3, 3, deltaMatrix);

        double[][] finalSolve = multiplyMatrix(3, 3, partialSolve, 3, 1, robotDelta);

        double deltaX = finalSolve[0][0];
        double deltaY = finalSolve[1][0];
        angle = newAngle;
        xpos += deltaX;
        ypos += deltaY;
        currentPose = new Pose2d(xpos, ypos, angle);
        double[] velo = {odomconst[0] * encoderPar.getVelocity() / ticks_per_inch, odomconst[1] * encoderBack.getVelocity() / ticks_per_inch};
        currentPOVVelocity = new Pose2d((velo[0] + velo[1]) * 0.5, velo[2] - angularVelocity.xRotationRate * LATERAL_OFFSET,
                angularVelocity.xRotationRate);
        currentVelocity = new Pose2d(currentPOVVelocity.vec().rotated(angle), currentPOVVelocity.getHeading());
        Canvas fieldOverlay = packet.fieldOverlay();
        packet.put("currentPose", currentPose);
        packet.put("currentVelocity", currentVelocity);
        packet.put("currentPOVVelocity", currentPOVVelocity);
        packet.put("parTicks", nowTicks[0]);
        packet.put("perpTicks", nowTicks[1]);

        if (currentPose != null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        angularVelocity = imu.getAngularVelocity();
        if (deltaAngle <= -180) //If the angle is -180, it should be 180, because they are at the same point. The acceptable angles are (-180, 180]
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        globalAngle %= 360;
        if (globalAngle > 270) {
            globalAngle -= 360;
        }
        if (globalAngle < -270) {
            globalAngle += 360;
        }

        lastAngles = angles;
        return globalAngle;
    }
}
