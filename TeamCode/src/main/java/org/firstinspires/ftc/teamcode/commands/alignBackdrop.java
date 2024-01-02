package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

//import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;

@Config
public class alignBackdrop implements Command {
    CrabRobot robot;
    DriveTrain mecanumDrive;
    RobotDistanceSensor distanceSensor;
    Telemetry telemetry;
    NanoClock clock;
    Pose2d drivePower;
    double startX, startY;
    double xPwr;
    public static double coef = 0.5; //0.5
    public static double hcoef = 0.0; //0.08

    public boolean isReached = false;
    double initialTimeStamp, intakeCompleteTime;
    double targetDis;

    //public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
    public alignBackdrop (CrabRobot robot, DriveTrain drive, Pose2d power, double xDisp, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        this.mecanumDrive = drive;
        this.distanceSensor = robot.ds;
        clock=NanoClock.system();
        drivePower= power;
        xPwr = drivePower.getX();
        targetDis = xDisp;
    }
    @Override
    public void start() {
        //mecanumDrive.setDrivePower(drivePower);
        initialTimeStamp=clock.seconds();
        startX = mecanumDrive.getPoseEstimate().getX();
        startY = mecanumDrive.getPoseEstimate().getY();
    }

    @Override
    public void update() {
        double dsL, dsR;

        dsL = distanceSensor.distanceLeft();
        dsR = distanceSensor.distanceRight();

        // L online, R not online => turn left
        if (Math.abs(dsL - dsR) > 1) {
            if(dsL > dsR) {
                //mecanumDrive.setDrivePower(new Pose2d(xPwr, xPwr * coef, -hcoef * xPwr));
                telemetry.addData("distL > distR, heading", -hcoef * xPwr);
            }
            else if(dsR > dsL) {
                //mecanumDrive.setDrivePower(new Pose2d(xPwr, -xPwr * coef, hcoef * xPwr));
                telemetry.addData("distL < distR, heading", hcoef * xPwr);
            }
        }else {
            if((dsL + dsR)/2 < targetDis-0.5) {
                //mecanumDrive.setDrivePower(new Pose2d(-xPwr,0,0));
                telemetry.addData("Back, actual dist ", (dsL + dsR)/2);
            }
            else if((dsL + dsR)/2 > targetDis+0.5){
                //mecanumDrive.setDrivePower(new Pose2d(xPwr,0,0));
                telemetry.addData("Forward, actual dist ", (dsL + dsR)/2);
            }
        }
        if(Math.abs((dsL + dsR)/2 - targetDis) <= 1 &&
                Math.abs(mecanumDrive.getPoseEstimate().getHeading()) <= Math.toRadians(2) ){

            //isReached = true;
        }
        telemetry.addData("distL: ",dsL);
        telemetry.addData("distR: ",dsR);
        telemetry.addData("Current heading: ",mecanumDrive.getPoseEstimate().getHeading());
        telemetry.update();
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