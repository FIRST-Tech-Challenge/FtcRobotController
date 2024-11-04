package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class DriveToPoint {

    private enum Direction {
        x,
        y,
        h;
    }

    private static double xyTolerance = 12;
    private static double yawTolerance = 0.0349066;

    private static double pGain = 0.03;
    private static double dGain = 0.002;
    private static double accel = 2.0;

    /*todo: make the forward/perpendicular pid values smarter. Instead of just using perpendicular for strafe. Use it for the smaller error axis, maybe a cutoff?
       consider if this is even better than a well tuned single PID loop*/

    private static double yawPGain = 5.0;
    private static double yawDGain = 0.0;
    private static double yawAccel = 2.0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private ElapsedTime holdTimer = new ElapsedTime();
    private ElapsedTime currentTime = new ElapsedTime();

    private PIDLoops xPID = new PIDLoops();
    private PIDLoops yPID = new PIDLoops();
    private PIDLoops hPID = new PIDLoops();

    private LinearOpMode myOpMode; //todo: consider if this is required

    public void setXYCoefficients(double p, double d, double acceleration, DistanceUnit unit, double tolerance){
        pGain = p;
        dGain = d;
        accel = acceleration;
        xyTolerance = unit.toMm(tolerance);
    }

    public void setYawCoefficients(double p, double d, double acceleration, AngleUnit unit, double tolerance){
        yawPGain = p;
        yawDGain = d;
        yawAccel = acceleration;
        yawTolerance = unit.toRadians(tolerance);
    }

    public DriveToPoint(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initializeMotors() {
        leftFrontDrive = setupDriveMotor("frontLeft", DcMotorSimple.Direction.REVERSE);
        leftBackDrive = setupDriveMotor("backLeft", DcMotorSimple.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("frontRight", DcMotorSimple.Direction.FORWARD);
        rightBackDrive = setupDriveMotor("backRight", DcMotorSimple.Direction.FORWARD);
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;
        double xPWR    = calculatePID(currentPosition,targetPosition,Direction.x);
        double yPWR    = calculatePID(currentPosition,targetPosition,Direction.y);
        double hOutput = calculatePID(currentPosition,targetPosition,Direction.h);

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine  = Math.cos(heading);
        double sine    = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        driveMecanums(xOutput*power, yOutput*power, hOutput*power);

        if(inBounds(currentPosition,targetPosition)){
            atTarget = true;
        }
        else {
            holdTimer.reset();
            atTarget = false;
        }

        if (atTarget && (holdTimer.time() > holdTime)){
            driveMecanums(0, 0, 0);
            return true;
        }
        return false;
    }

    private void driveMecanums(double forward, double strafe, double yaw) {
        double leftFront = forward - -strafe - yaw;
        double rightFront = forward + -strafe + yaw;
        double leftBack = forward + -strafe - yaw;
        double rightBack = forward - -strafe + yaw;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return aMotor;
    }


    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction){
        if(direction == Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, pGain, dGain, accel, currentTime.time());
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, pGain, dGain, accel, currentTime.time());
        }
        if(direction == Direction.h){
            double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            return hPID.calculateAxisPID(hError, yawPGain, yawDGain, yawAccel, currentTime.time());
        }
        return 0;
    }

    private boolean inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xOutOfBounds = currPose.getX(MM) > (trgtPose.getX(MM) - xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + xyTolerance);
        boolean yOutOfBounds = currPose.getY(MM) > (trgtPose.getY(MM) - xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + xyTolerance);
        boolean hOutOfBounds = currPose.getHeading(RADIANS) > (trgtPose.getHeading(RADIANS) - yawTolerance) &&
                currPose.getHeading(RADIANS) < (trgtPose.getHeading(RADIANS) + yawTolerance);

        return xOutOfBounds && yOutOfBounds && hOutOfBounds;
    }
}

class PIDLoops{
    private double previousError;
    private double previousTime;
    private double previousOutput;

    double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime){
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double d = dGain * (previousError - error) / (cycleTime);
        double output = p+d;
        double dV = cycleTime * accel;

        double max = Math.abs(output);
        if(max > 1.0){
            output /= max;
        }

        if((output - previousOutput) > dV){
            output = previousOutput + dV;
        } else if ((output - previousOutput) < -dV){
            output = previousOutput - dV;
        }

        previousOutput = output;
        previousError  = error;
        previousTime   = currentTime;

        return output;
    }
}

