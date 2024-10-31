package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DriveBaseSubsystem {
    public static float Translational_P = 0.07f;
    public static float Translational_I = 0.000007f; // 0.000005 is good
    public static float Translational_D = 0;
    // Motors & Sensors
    private static DcMotor _frontLeftMotor;
    private static DcMotor _frontRightMotor;
    private static DcMotor _backLeftMotor;
    private static DcMotor _backRightMotor;
    private static SparkFunOTOS _odometry;

    // Position Controllers
    private final PIDController translationX = new PIDController();
    private final PIDController translationY = new PIDController();
    private final PIDController headingPID = new PIDController();

    // Pose Vars
    private static SparkFunOTOS.Pose2D currentPose;
    private static SparkFunOTOS.Pose2D previousPose;
    private static Long timeAtLastCycle;
    private static SparkFunOTOS.Pose2D goal;
    private static Telemetry telemetry;

    public DriveBaseSubsystem(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, SparkFunOTOS odometry, Telemetry theTelemetry) {
        // Motors and Sensors
        _frontLeftMotor = frontLeftMotor;
        _frontRightMotor = frontRightMotor;
        _backLeftMotor = backLeftMotor;
        _backRightMotor = backRightMotor;
        _frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _odometry = odometry;

        // PID Stuff
        translationX.init(Translational_P, Translational_I, Translational_D);
        translationY.init(Translational_P, Translational_I, Translational_D);
        headingPID.init(0.02f, 0, 0.015f);

        // Make sure odo is ready
        _odometry.begin();
        _odometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));

        // Set Units
        _odometry.setAngularUnit(AngleUnit.DEGREES);
        _odometry.setLinearUnit(DistanceUnit.INCH);
        _odometry.resetTracking();

        currentPose = new SparkFunOTOS.Pose2D();
        previousPose = new SparkFunOTOS.Pose2D();

        currentPose.set(odometry.getPosition());
        previousPose.set(odometry.getPosition());
        timeAtLastCycle = System.currentTimeMillis();
        telemetry = theTelemetry;
        telemetry.addData("CurrentPoseX", currentPose.x);
        telemetry.addData("CurrentPoseY", currentPose.y);
        telemetry.addData("CurrentPoseAngle", currentPose.h);
        telemetry.addData("GoalPoseX", currentPose.x);
        telemetry.addData("GoalPoseY", currentPose.y);
        telemetry.addData("GoalPoseAngle", currentPose.h);
        telemetry.update();
        previousPose.set(currentPose);
        timeAtLastCycle = System.currentTimeMillis();
    }

    public void setGoal(SparkFunOTOS.Pose2D Goal) {
        goal = Goal;
    }

    private double motorClamp(double value) {
        double max = 0.8;
        if (value > max) {
            return max;
        } else if (value < -max) {
            return -max;
        } else {
            return value;
        }
    }

    private void driveMech(double x, double y, double rx, double heading, TelemetryPacket packet) {
        packet.put("XPower", x);
        packet.put("YPower", y);
        packet.put("RXPower", rx);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = (x * Math.sin(-heading) + y * Math.cos(-heading)) * 1.1; // Strafe Helper
        _frontLeftMotor.setPower(motorClamp(rotY + rotX + rx));
        _backLeftMotor.setPower(motorClamp(rotY - rotX + rx));
        _frontRightMotor.setPower(motorClamp(rotY - rotX - rx));
        _backRightMotor.setPower(motorClamp(rotY + rotX - rx));
    }

    void periodicUpdate(TelemetryPacket packet) {
        currentPose.set(_odometry.getPosition());
        driveMech(-translationX.getOutput((float) currentPose.y, (float) goal.x, packet, "transX"), translationY.getOutput((float) currentPose.x, (float) goal.y, packet, "transY"), headingPID.getOutput((float) -currentPose.h, (float) goal.h, packet, "head"), Math.toRadians(currentPose.h), packet);
        telemetry.addData("Current Pose", "(" + currentPose.x + ", " + currentPose.y + ", " + currentPose.x + ")");
        telemetry.addData("Goal Pose", "(" + goal.x + ", " + goal.y + ", " + goal.x + ")");
        packet.fieldOverlay().fillRect(currentPose.y, currentPose.x, 18,18);
        packet.put("x", currentPose.y);
        packet.put("y", currentPose.x);
        packet.put("angle", -currentPose.h);
        packet.put("Goalx", goal.x);
        packet.put("Goaly", goal.y);
        packet.put("GoalAngle", goal.h);
        previousPose.set(currentPose);
        timeAtLastCycle = System.currentTimeMillis();
    }

    public boolean isAtReference(TelemetryPacket packet, double time) {
        boolean x, y, h;
        if (time > 20000) {
            x = ((goal.x-2) < currentPose.y) && (currentPose.y < (goal.x + 2));
            y = ((goal.y-2) < currentPose.x) && (currentPose.x < (goal.y + 2));
            h = Math.abs(Math.abs(currentPose.h) - Math.abs(goal.h)) < 6;
        } else {
            x = ((goal.x-1) < currentPose.y) && (currentPose.y < (goal.x + 1));
            y = ((goal.y-1) < currentPose.x) && (currentPose.x < (goal.y + 1));
            h = Math.abs(Math.abs(currentPose.h) - Math.abs(goal.h)) < 4;
        }
//        boolean h = ((goal.x - 2) < -currentPose.h) && (-currentPose.h < (goal.h + 2));
        packet.put("xAcceptable", x);
        packet.put("yAcceptable", y);
        packet.put("hAcceptable", h);
        return  x && y && h;
    }
}