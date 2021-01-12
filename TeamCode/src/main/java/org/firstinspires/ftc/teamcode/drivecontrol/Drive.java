package org.firstinspires.ftc.teamcode.drivecontrol;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.teamcode.drivecontrol.Module.ModuleRotationMode.ROTATION_DISABLED;
import static org.firstinspires.ftc.teamcode.drivecontrol.Module.ModuleRotationMode.ROTATION_ENABLED;

public class Drive {
    private boolean debug;

    public enum ModuleSide {
        LEFT, RIGHT
    }

    private static final double MIN_POWER = 0;
    private static final double SLOWDOWN_DISTANCE = 0;
    private static final double DEFAULT_TIMEOUT_ROT_MODULES = 750;
    private static final double WHEEL_TO_WHEEL_CM = 32.5;

    private Module moduleLeft;
    private Module moduleRight;

    private Pos pos;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private Log log;
    private double robotDistanceTraveled = 0;
    private double moduleLeftLastDistance = 0;
    private double moduleRightLastDistance = 0;

    public boolean isRunning = true;

    public Drive(boolean debugMode, Pos startingPosition, Module left, Module right, BNO055IMU imu, Telemetry telemetry) {
        debug = debugMode;

        pos = startingPosition;
        this.imu = imu;
        this.telemetry = telemetry;

        moduleLeft = left;
        moduleRight = right;

        if (debug) {
            log = new Log("Drive Controller");
            log.addField("X Position");
            log.addField("Y Position");
            log.addField("X Power");
            log.addField("Y Power");
            log.addField("Rotation Power");
            log.addField("Translation Direction X");
            log.addField("Translation Direction Y");
            log.addField("Rotation Direction");
            log.addField("Translation Vector X");
            log.addField("Translation Vector Y");
            log.newLine();
        }
    }

    private void update(Vector2D translationVector, double rotationMagnitude, double heading) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude, heading);
        moduleRight.updateTarget(translationVector, rotationMagnitude, heading);
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
                speed = Utilities.scaleDouble(distance - getDistances(), 0, SLOWDOWN_DISTANCE, MIN_POWER, initialSpeed);
            }
            update(direction.normalize(Math.abs(speed)), 0, getHeading());
        }
        update(Vector2D.ZERO, 0, getHeading());
        setModuleRotationMode(ROTATION_ENABLED);
    }

    private double getDistances() {
        return Math.abs(robotDistanceTraveled);
    }

    private void resetDistances() {
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    private void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    private void updateSLAM() { // TODO
//        T265Camera slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);;
//        private final FtcDashboard dashboard = FtcDashboard.getInstance();
//        final int robotRadius = 9; // inches
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas field = packet.fieldOverlay();
//        T265Camera.CameraUpdate up;
//        Translation2d translationSLAM;
//        Rotation2d rotationSLAM;
//
//
//        up = slamra.getLastReceivedCameraUpdate();
//        translationSLAM = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
//        rotationSLAM = up.pose.getRotation();
//
//        field.strokeCircle(translationSLAM.getX(), translationSLAM.getY(), robotRadius);
//        double arrowX = rotationSLAM.getCos() * robotRadius, arrowY = rotationSLAM.getSin() * robotRadius;
//        double x1 = translationSLAM.getX() + arrowX  / 2, y1 = translationSLAM.getY() + arrowY / 2;
//        double x2 = translationSLAM.getX() + arrowX, y2 = translationSLAM.getY() + arrowY;
//        field.strokeLine(x1, y1, x2, y2);
//
//        dashboard.sendTelemetryPacket(packet);
    }

    private void setModuleRotationMode(Module.ModuleRotationMode mode) {
        moduleLeft.setRotationMode(mode);
        moduleRight.setRotationMode(mode);
    }

    private void rotateModules(Vector2D direction, boolean fieldCentric, double timeoutMS) {
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            updateSLAM();
            updateTracking();
            moduleLeftDifference = getDifference(moduleLeft.getCurrentOrientation(), direction.getAngle());
            moduleRightDifference = getDifference(moduleRight.getCurrentOrientation(), direction.getAngle());
            moduleLeft.rotateModule(direction, fieldCentric, getHeading());
            moduleRight.rotateModule(direction, fieldCentric, getHeading());

            telemetry.addData("Rotating MODULES", "");
            telemetry.addData("Top level module left difference", moduleLeftDifference);
            telemetry.addData("Top level module right difference", moduleRightDifference);
            telemetry.update();
            updatePositionTracking(); //update position tracking
        } while ((moduleLeftDifference > Module.ORIENTATION_ERROR_MARGIN || moduleRightDifference > Module.ORIENTATION_ERROR_MARGIN) && isRunning && System.currentTimeMillis() < startTime + timeoutMS);
        update(Vector2D.ZERO, 0, getHeading());
    }

    private double getDifference(double a1, double a2) {
        double raw = a1 - a2;
        if (raw > 180) {
            return 360 - raw;
        }
        return raw;
    }

    private void updatePositionTracking() {
        Vector2D rightDisplacement = moduleRight.updatePositionTracking();
        Vector2D leftDisplacement = moduleLeft.updatePositionTracking();

        double arcLength = moduleRight.positionChange - moduleLeft.positionChange;
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

    private double getHeading() {
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        return raw % 360;
    }
}
