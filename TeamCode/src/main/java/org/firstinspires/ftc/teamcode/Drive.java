package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Module.ModuleRotationMode.ROTATION_DISABLED;
import static org.firstinspires.ftc.teamcode.Module.ModuleRotationMode.ROTATION_ENABLED;

public class Drive {
    private boolean debug;

    public enum ModuleSide {
        LEFT, RIGHT
    }

    private static final double MIN_POWER = 0;
    private static final double SLOWDOWN_DISTANCE = 0;

    private Module moduleLeft;
    private Module moduleRight;

    private Pos pos;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private Log log;
    private double robotDistanceTraveled = 0;
    private double moduleLeftLastDistance = 0;
    private double moduleRightLastDistance = 0;

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
            rotateModules(direction);
        }

        if (noRotate) {
            setModuleRotationMode(ROTATION_DISABLED);
        } else {
            setModuleRotationMode(ROTATION_ENABLED);
        }
        resetDistances();
        updateTracking();
        while (getDistances() < distance) {
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

    private void rotateModules(Vector2D direction) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            updateSLAM();
            updateTracking();
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngle()); //was getRealAngle() (don't ask)
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngle());
            moduleLeft.rotateModule(direction, fieldCentric);
//            moduleLeft.rotateModule(direction, fieldCentric);
            moduleRight.rotateModule(direction, fieldCentric);

            telemetry.addData("Rotating MODULES", "");
            telemetry.addData("Top level module left difference", moduleLeftDifference);
            telemetry.addData("Top level module right difference", moduleRightDifference);
            telemetry.update();
            updatePositionTracking(); //update position tracking
        } while ((moduleLeftDifference > Module.ORIENTATION_ERROR_MARGIN || moduleRightDifference > Module.ORIENTATION_ERROR_MARGIN) && linearOpMode.opModeIsActive() && System.currentTimeMillis() < startTime + timemoutMS);
        update(Vector2D.ZERO, 0, getHeading());
    }

    private double updatePositionTracking() {
        Vector2d rightDisp = moduleRight.updatePositionTracking(telemetry);
        Vector2d leftDisp = moduleLeft.updatePositionTracking(telemetry);
    }

    private double getHeading() {
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        return raw % 360;
    }
}
