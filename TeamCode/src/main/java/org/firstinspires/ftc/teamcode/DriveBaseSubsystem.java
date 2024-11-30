package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

@Config
public class DriveBaseSubsystem {
    public static float Translational_P = 0.005f;
    public static float Translational_I = 0f; // 0.000005 is good
    public static float Translational_D = 0.035f;
    public static float Translational_Strafe_P = 0.09f;
    public static float Translational_Strafe_I = 0f; // 0.000005 is good
    public static float Translational_Strafe_D = 0.03f;
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
        translationX.init(Translational_Strafe_P, Translational_Strafe_I, Translational_Strafe_D);
        translationY.init(Translational_P, Translational_I, Translational_D);
        headingPID.init(0.01f, 0, 0);

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
        if (Goal.h < 0) {
            throw new Error("Incompatible angle value: Goal Angle must be greater than 0");
        }
        if (Goal.h > 360) {
            throw new Error("Incompatible angle value: Goal Angle must be less than 360");
        }
        goal = Goal;
    }

    public void changePID(float p, float i, float d) {
        Translational_P = p;
        Translational_I = i;
        Translational_D = d;
        // PID Stuff
        translationX.resetPID(p, i, d);
        translationY.resetPID(p, i, d);
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
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = (x * Math.sin(-heading) + y * Math.cos(-heading)) * 1.1; // Strafe Helper
        packet.put("XPower", rotX);
        packet.put("YPower", rotY);
        packet.put("RXPower", rx);
        _frontLeftMotor.setPower(motorClamp(rotY + rotX + rx));
        _backLeftMotor.setPower(motorClamp(rotY - rotX + rx));
        _frontRightMotor.setPower(motorClamp(rotY - rotX - rx));
        _backRightMotor.setPower(motorClamp(rotY + rotX - rx));
    }

    public double XY(double CurrentPose, double GoalPose) {
        double error = GoalPose - CurrentPose;
        if (error < 0) {
            return (-0.2 * Math.sqrt(Math.abs(error))) + (0.005 * error);
        } else {
            return (0.2 * Math.sqrt(error)) + (0.005 * error);
        }
    }
    public double Rotation(double CurrentPose, double GoalPose) {
        if (GoalPose < 0) {
            throw new Error("Incompatible angle value: Goal Angle must be greater than 0");
        }
        if (GoalPose > 360) {
            throw new Error("Incompatible angle value: Goal Angle must be less than 360");
        }
        if (GoalPose == 360) {
            GoalPose = 0;
        }
        double turnRightError = (CurrentPose + 360) - GoalPose;
        double turnLeftError = GoalPose - CurrentPose;
        if (turnRightError < turnLeftError) {
            if (turnLeftError < 0) {
                return -1 * ((-0.01 * Math.sqrt(Math.abs(turnRightError))) + (0.00224 * turnRightError));
            } else {
                return -1 * ((0.01 * Math.sqrt(turnRightError)) + (0.00224 * turnRightError));
            }
        } else {
            if (turnRightError < 0) {
                return (-0.01 * Math.sqrt(Math.abs(turnLeftError))) + (0.00224 * turnLeftError);
            } else {
                return (0.01 * Math.sqrt(turnLeftError)) + (0.00224 * turnLeftError);
            }
        }
    }

    void periodicUpdate(TelemetryPacket packet) {
        // Read Current Pose
        currentPose.set(_odometry.getPosition());

        // Deal With Rotation
        double rxValue;
        if (goal.h < 0) {
            throw new Error("Incompatible angle value: Goal Angle must be greater than 0");
        }
        if (goal.h > 360) {
            throw new Error("Incompatible angle value: Goal Angle must be less than 360");
        }
        if (goal.h == 360) {
            goal.h = 0;
        }
        double normalError = Math.abs(goal.h - currentPose.h);
        double turnRightError = Math.abs((goal.h + 360) - currentPose.h);
        double turnLeftError = Math.abs((currentPose.h + 360) - goal.h);
        if (turnRightError > turnLeftError) {
            if (normalError < turnLeftError) {
                rxValue = headingPID.getOutput((float) currentPose.h, (float) goal.h, packet, "heading");
            } else {
                rxValue = -1 * Math.abs(headingPID.getOutput((float) goal.h, (float) (currentPose.h + 360), packet, "heading"));
            }
        } else {
            if (normalError < turnRightError) {
                rxValue = headingPID.getOutput((float) currentPose.h, (float) goal.h, packet, "heading");
            } else {
                rxValue = Math.abs(headingPID.getOutput((float) currentPose.h, (float) goal.h, packet, "heading"));
            }
        }

        // Drive Mechanum
        driveMech(
                -translationX.getOutput((float) -currentPose.y, (float) goal.x, packet, "transX"),
                translationY.getOutput((float) -currentPose.x, (float) goal.y, packet, "transY"),
                -rxValue,
                Math.toRadians(currentPose.h),
                packet
        );

        // Send Telemetry
        telemetry.addData("Current Pose", "(" + -currentPose.x + ", " + currentPose.y + ", " + currentPose.h + ")");
        telemetry.addData("Goal Pose", "(" + goal.x + ", " + goal.y + ", " + goal.x + ")");
        // driveMech(XY(currentPose.x, goal.x), XY(currentPose.y, goal.y), Rotation(currentPose.h, goal.h), Math.toRadians(currentPose.h), packet);
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
            x = ((goal.x-2) < -currentPose.y) && (-currentPose.y < (goal.x + 2));
            y = ((goal.y-2) < -currentPose.x) && (-currentPose.x < (goal.y + 2));
            h = Math.abs(Math.abs(currentPose.h) - Math.abs(goal.h)) < 6;
        } else {
            x = ((goal.x-1) < -currentPose.y) && (-currentPose.y < (goal.x + 1));
            y = ((goal.y-1) < -currentPose.x) && (-currentPose.x < (goal.y + 1));
            h = Math.abs(Math.abs(currentPose.h) - Math.abs(goal.h)) < 4;
        }
//        boolean h = ((goal.x - 2) < -currentPose.h) && (-currentPose.h < (goal.h + 2));
        packet.put("xAcceptable", x);
        packet.put("yAcceptable", y);
        packet.put("hAcceptable", h);
        return  x && y && h;
    }
}