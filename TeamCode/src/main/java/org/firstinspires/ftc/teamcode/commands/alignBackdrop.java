package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

//import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
import com.acmerobotics.roadrunner.control.PIDFController;

@Config
public class alignBackdrop implements Command {
    CrabRobot robot;
    DriveTrain mecanumDrive;
    RobotDistanceSensor distanceSensor;
    Telemetry telemetry;
    NanoClock clock;
    Pose2d drivePower;
    double startX, startY;
    double xPwr, hcoef;

    public static double DIST_TOL = 1; //cm
    public static double HEAD_TOL = 5; //degree

    public static double SENSOR_DIST = 17; //cm

    public boolean isReached = false;
    private PIDFController pidfController;

    public static double kP = 0.15;
    public static double kI = 0.0; //0.0000000001;
    public static double kD = 0.0;
    public static double kA = 0.0;
    public static double kV = 0.0;
    public static double kS = 0.002;
    public static double PID_RANGE = 0.9;
    private double powerFromPIDF;
    private double powerFromHeadingPID;

    private PIDFController headingPID;
    public static double headingKp = 0.15;
    public static double headingKs = 0.002;
    public static double HEADING_PID_RANGE = 0.9;
    double initialTimeStamp, intakeCompleteTime;
    double targetDis;

    //public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
    public alignBackdrop(CrabRobot robot, DriveTrain drive, double distCoeff, double hCoef, double xDisp, Telemetry telemetry) {
        // PID related
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.kP = kP;
        coefficients.kI = kI;
        coefficients.kD = kD;
        pidfController = new PIDFController(coefficients, kV, kA, kS);
        pidfController.setOutputBounds(-1.0 * PID_RANGE, 1.0 * PID_RANGE);
        pidfController.reset();

        PIDCoefficients headingCoeff = new PIDCoefficients();
        headingCoeff.kP = headingKp;
        headingCoeff.kI = 0;
        headingCoeff.kD = 0;
        headingPID = new PIDFController(headingCoeff, 0, 0, headingKs);
        headingPID.setOutputBounds(-1.0 * HEADING_PID_RANGE, 1.0 * HEADING_PID_RANGE);
        headingPID.reset();

        // Command related
        this.robot = robot;
        this.telemetry = telemetry;
        this.mecanumDrive = drive;
        this.distanceSensor = robot.ds;
        clock = NanoClock.system();
        xPwr = distCoeff;
        hcoef = hCoef;
        targetDis = xDisp;
    }

    @Override
    public void start() {
        //mecanumDrive.setDrivePower(drivePower);
        initialTimeStamp = clock.seconds();
        startX = mecanumDrive.getPoseEstimate().getX();
        startY = mecanumDrive.getPoseEstimate().getY();
        pidfController.reset();
        pidfController.setTargetPosition(targetDis);
        headingPID.reset();
        headingPID.setTargetPosition(0);
    }

    @Override
    public void update() {
        double dsL, dsR;

        dsL = distanceSensor.distanceLeft();
        dsR = distanceSensor.distanceRight();
        //telemetry.addData("distL ", dsL);
        //telemetry.addData("distR ", dsR);

        // Distance PID
        double measuredPosition = (double) (dsL + dsR) / 2;
        powerFromPIDF = -xPwr * pidfController.update(measuredPosition);
        //telemetry.addData("forward Pwr ", powerFromPIDF);

        // Heading PID
        double measuredHeading = Math.asin(Math.abs(dsR-dsL)/SENSOR_DIST);
        //telemetry.addData("measuredHeading ", measuredHeading);
        int headingSign = (dsL > dsR) ? -1 : 1;
        //telemetry.addData("headingSign ", headingSign);
        powerFromHeadingPID = Math.abs(headingPID.update(measuredHeading));
        //telemetry.addData("powerFromHeadingPIO ", powerFromHeadingPID);
        powerFromHeadingPID *= hcoef;
        //telemetry.addData("powerFromHeadingPID after scaling ", powerFromHeadingPID);
        //telemetry.addData("heading Pwr ", powerFromHeadingPID*headingSign);
        //telemetry.update();

        if(Math.abs(measuredPosition - targetDis) <= DIST_TOL
                && measuredHeading <= Math.toRadians(HEAD_TOL)){
            isReached = true;
        } else {
            mecanumDrive.setDrivePower(new Pose2d(powerFromPIDF, 0, powerFromHeadingPID*headingSign));
        }


    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d(0,0,0));
    }

    @Override
    public boolean isCompleted() {

        //return (disp >= driveDisp);
        return isReached;


    }
}