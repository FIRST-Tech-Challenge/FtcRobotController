package org.firstinspires.ftc.teamcode.robots.LilVirani.subsystem;


import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.robots.LilVirani.util.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

@Config(value = "FFCrane")
public class DriveTrain implements Subsystem {




    private double targetHeading, targetVelocity = 0;

    public Pose2d currentPose;
    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;


    //devices ---------------------------------------------------------

    public DcMotorEx leftMotor = null;
    public DcMotorEx rightMotor = null;

    private final BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private double compensatedBatteryVoltage;

    //PID LOOPS_______________________________________________________________________

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
    public static double HEADING_PID_TOLERANCE = 1;
    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(4, 0, 0);

    public static PIDController headingPID;
    public static PIDController velocityPID;

    public DriveTrain (HardwareMap hardwareMap){





            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

            List<DcMotorEx> motors = Arrays.asList(leftMotor, rightMotor);

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);

                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                compensatedBatteryVoltage = batteryVoltageSensor.getVoltage();

            }



            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);




        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(0, Math.toRadians(360));
        headingPID.setOutputRange(-100, 100);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

        velocityPID = new PIDController(VELOCITY_PID);
        velocityPID.setInputRange(0, Math.toRadians(360));
        velocityPID.setOutputRange(-100, 100);
        velocityPID.setContinuous(true);
        velocityPID.setTolerance(HEADING_PID_TOLERANCE);
        velocityPID.enable();                                                                                     



        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);
        currentPose = new Pose2d(0,0,0);

        //TODO, starting pose


    }
    public double updateHeading(double dtheta){
        //TODO Mix IMU data with encoder data
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);

        return orientation.firstAngle;
    }

    public void setTargetVelocity(double vel){
        velocityPID.setSetpoint(vel);
    }

    public void setTargetHeading(double vel){
        headingPID.setSetpoint(vel);
    }

    public void updatePose(){
        double dLeft = diffEncoderTicksToInches(leftMotor.getCurrentPosition());
        double dRight = diffEncoderTicksToInches(rightMotor.getCurrentPosition());

        if(dLeft == dRight) {
            //in this case, robot just moves foreward, and just use simple trig to update pose
            currentPose = new Pose2d(currentPose.getX() + dLeft*Math.sin(currentPose.getHeading()),
                    currentPose.getY() + dRight*Math.cos(currentPose.getHeading()));
            return;
        }
        if((dLeft > 0 && dRight > 0) || (dLeft < 0 && dRight < 0)){

        // in this case, robot is turning with both motors in the same direction.

                double R = Constants.DISTANCE_BETWEEN_WHEELS / (1 - Math.min(dLeft / dRight, dRight/dLeft));
                double midpointRadius = R - Constants.DISTANCE_BETWEEN_WHEELS/2;

                double thetaModifier = (Math.abs(dRight) > Math.abs(dLeft) ? 0 : Math.PI);
                double x1 = Math.cos(currentPose.getHeading() + thetaModifier)*midpointRadius;
                double y1 = Math.sin(currentPose.getHeading() + thetaModifier)*midpointRadius;

                double newTheta = updateHeading(R/Math.max(Math.abs(dRight), Math.abs(dLeft)));
                //if turning left, we use normal theta
                //if turning right, we use theta + pi

                double x2 = Math.cos(newTheta + thetaModifier) * midpointRadius;
                double y2 = Math.sin(newTheta + thetaModifier) * midpointRadius;
                currentPose = new Pose2d( currentPose.getX() + (x2 - x1),
                        currentPose.getY() + (y2 - y1), newTheta);
                return;

        }
        if (dLeft != 0 && dRight != 0) {
            //r + l = L
            //r = dr/dl l

            double rLeft = Constants.DISTANCE_BETWEEN_WHEELS / (1 + Math.abs(dRight/dLeft));
            // if rLeft is greater than half the distance between the wheels, the center of the circle is to the right
            // if rLeft is less than half the distance between the wheels, the center of the circle is to the left.
            double rRight = Constants.DISTANCE_BETWEEN_WHEELS - rLeft;
            double midpointRadius = Math.max(rLeft, rRight) - Constants.DISTANCE_BETWEEN_WHEELS/2;


            double thetaModifier = (rRight > rLeft ? 0 : Math.PI);
            double x1 = Math.cos(currentPose.getHeading() + thetaModifier)*midpointRadius;
            double y1 = Math.sin(currentPose.getHeading() + thetaModifier)*midpointRadius;

            double newTheta = updateHeading(Math.abs(rRight/dRight));
            double x2 = Math.cos(currentPose.getHeading() + thetaModifier)*midpointRadius;
            double y2 = Math.sin(currentPose.getHeading() + thetaModifier)*midpointRadius;
            currentPose = new Pose2d( currentPose.getX() + (x2 - x1),
                    currentPose.getY() + (y2 - y1), newTheta);
            return;


        }
        else {

            double midpointRadius = Constants.DISTANCE_BETWEEN_WHEELS/2;


            double thetaModifier = (dLeft == 0 ? 0 : Math.PI);
            double x1 = Math.cos(currentPose.getHeading() + thetaModifier)*midpointRadius;
            double y1 = Math.sin(currentPose.getHeading() + thetaModifier)*midpointRadius;

            double newTheta = updateHeading(Math.max(Math.abs(dRight), Math.abs(dLeft)) / Constants.DISTANCE_BETWEEN_WHEELS);
            double x2 = Math.cos(currentPose.getHeading() + thetaModifier)*midpointRadius;
            double y2 = Math.sin(currentPose.getHeading() + thetaModifier)*midpointRadius;
            currentPose = new Pose2d( currentPose.getX() + (x2 - x1),
                    currentPose.getY() + (y2 - y1), newTheta);
            return;


        }

    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePose();
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
