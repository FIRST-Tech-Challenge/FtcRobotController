package org.firstinspires.ftc.teamcode.drivecontrol;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.drivecontrol.WheelModule.ModuleRotationMode.ROTATION_DISABLED;
import static org.firstinspires.ftc.teamcode.drivecontrol.WheelModule.ModuleRotationMode.ROTATION_ENABLED;

public class DriveController {
    public enum ModuleSide {
        LEFT, RIGHT
    }

    private static final double MIN_POWER = 0;
    private static final double SLOWDOWN_DISTANCE = 0;
    private static final double DEFAULT_TIMEOUT_ROT_MODULES = 750;
    private static final double WHEEL_TO_WHEEL_CM = 32.5;
    private static final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;
    private static final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR = 0.2;

    private WheelModule leftWheelModule;
    private WheelModule rightWheelModule;

    private Position pos;
    private final BNO055IMU imu;
    private final Telemetry telemetry;
    private Logger logger = null;
    private double robotDistanceTraveled = 0;
    private double moduleLeftLastDistance = 0;
    private double moduleRightLastDistance = 0;

    private final T265Camera slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();
    private final Canvas field = packet.fieldOverlay();

    public boolean isRunning = true;

    public DriveController(boolean debugMode, Position startingPosition, WheelModule left, WheelModule right, BNO055IMU imu, Telemetry telemetry) {
        pos = startingPosition;
        this.imu = imu;
        this.telemetry = telemetry;

        leftWheelModule = left;
        rightWheelModule = right;

        if (debugMode) {
            logger = new Logger("Drive Controller");
            logger.addField("X Position");
            logger.addField("Y Position");
            logger.addField("X Power");
            logger.addField("Y Power");
            logger.addField("Rotation Power");
            logger.addField("Translation Direction X");
            logger.addField("Translation Direction Y");
            logger.addField("Rotation Direction");
            logger.addField("Translation Vector X");
            logger.addField("Translation Vector Y");
            logger.newLine();
        }
    }

    private void update(Vector2D translationVector, double rotationMagnitude) {
        leftWheelModule.updateTarget(translationVector, rotationMagnitude, getHeading());
        rightWheelModule.updateTarget(translationVector, rotationMagnitude, getHeading());
    }

    public void drive(Vector2D direction, double speed, double distance, boolean noRotate, boolean align) {
        double initialSpeed = speed;

        if (align) {
            rotateModules(direction, false, DEFAULT_TIMEOUT_ROT_MODULES);
        }

        if (noRotate) {
            setModuleRotationMode(ROTATION_DISABLED);
        } else {
            setModuleRotationMode(ROTATION_ENABLED);
        }
        resetDistances();
        updateTracking();
        while (getDistances() < distance && isRunning) {
            updateSLAM();
            updateTracking();
            if (distance - getDistances() < SLOWDOWN_DISTANCE) {
                speed = RobotUtilities.scaleDouble(distance - getDistances(), 0, SLOWDOWN_DISTANCE, MIN_POWER, initialSpeed);
            }
            update(direction.normalize(Math.abs(speed)), 0);
        }
        update(Vector2D.ZERO, 0);
        setModuleRotationMode(ROTATION_ENABLED);
    }

    private double getDistances() {
        return Math.abs(robotDistanceTraveled);
    }

    private void resetDistances() {
        robotDistanceTraveled = 0;

        rightWheelModule.resetDistanceTraveled();
        leftWheelModule.resetDistanceTraveled();

        moduleLeftLastDistance = leftWheelModule.getDistanceTraveled();
        moduleRightLastDistance = rightWheelModule.getDistanceTraveled();
    }

    private void updateTracking() {
        rightWheelModule.updateTracking();
        leftWheelModule.updateTracking();

        double moduleLeftChange = leftWheelModule.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = rightWheelModule.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = leftWheelModule.getDistanceTraveled();
        moduleRightLastDistance = rightWheelModule.getDistanceTraveled();
    }

    private void updateSLAM() { // TODO check functionality
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        Translation2d translationSLAM = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotationSLAM = up.pose.getRotation();
        int robotRadius = 9;

        field.strokeCircle(translationSLAM.getX(), translationSLAM.getY(), robotRadius);
        double arrowX = rotationSLAM.getCos() * robotRadius, arrowY = rotationSLAM.getSin() * robotRadius;
        double x1 = translationSLAM.getX() + arrowX  / 2, y1 = translationSLAM.getY() + arrowY / 2;
        double x2 = translationSLAM.getX() + arrowX, y2 = translationSLAM.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }

    private void setModuleRotationMode(WheelModule.ModuleRotationMode mode) {
        leftWheelModule.setRotationMode(mode);
        rightWheelModule.setRotationMode(mode);
    }

    private void rotateModules(Vector2D direction, boolean fieldCentric, double timeoutMS) {
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            updateSLAM();
            updateTracking();
            moduleLeftDifference = getDifference(leftWheelModule.getCurrentOrientation(), direction.getAngle());
            moduleRightDifference = getDifference(rightWheelModule.getCurrentOrientation(), direction.getAngle());
            leftWheelModule.rotateModule(direction, fieldCentric, getHeading());
            rightWheelModule.rotateModule(direction, fieldCentric, getHeading());

            telemetry.addData("Rotating MODULES", "");
            telemetry.addData("Top level module left difference", moduleLeftDifference);
            telemetry.addData("Top level module right difference", moduleRightDifference);
            telemetry.update();
            updatePositionTracking(); //update position tracking
        } while ((moduleLeftDifference > WheelModule.ORIENTATION_ERROR_MARGIN || moduleRightDifference > WheelModule.ORIENTATION_ERROR_MARGIN) && isRunning && System.currentTimeMillis() < startTime + timeoutMS);
        update(Vector2D.ZERO, 0);
    }

    private double getDifference(double a1, double a2) {
        double raw = a1 - a2;
        if (raw > 180) {
            return 360 - raw;
        }
        return raw;
    }

    public void updatePositionTracking() {
        Vector2D rightDisplacement = rightWheelModule.updatePositionTracking();
        Vector2D leftDisplacement = leftWheelModule.updatePositionTracking();

        double arcLength = rightWheelModule.positionChange - leftWheelModule.positionChange;
        double angleChange = arcLength * 360 / 2.0 / Math.PI / WHEEL_TO_WHEEL_CM;
        pos.incrementHeading(angleChange);

        rightDisplacement.setX(rightDisplacement.getX() + WHEEL_TO_WHEEL_CM /2);
        leftDisplacement.setX(leftDisplacement.getX() - WHEEL_TO_WHEEL_CM /2);

        Vector2D robotCenterDisp = new Vector2D((rightDisplacement.getX() + leftDisplacement.getX())/2, (rightDisplacement.getY() + leftDisplacement.getY())/2);
        robotCenterDisp = robotCenterDisp.rotateBy(pos.getHeading(), Vector2D.CW);
        pos.incrementX(robotCenterDisp.getX());
        pos.incrementY(robotCenterDisp.getY());

        telemetry.addData("Robot X Position: ", pos.getX());
        telemetry.addData("Robot Y Position: ", pos.getY());
        telemetry.addData("Robot Abs Heading: ", pos.getHeading());
    }

    public void updateUsingJoysticks(Vector2D j1, Vector2D j2) {
        if (j1.getMagnitude() == 0) {
            update(j1, -j2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
        } else {
            update(j1, -j2.getX() * ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR);
        }
    }

    private double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle % 360;
    }
}
