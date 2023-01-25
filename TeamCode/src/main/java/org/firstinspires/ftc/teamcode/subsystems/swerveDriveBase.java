package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.isNaN;

import org.firstinspires.ftc.teamcode.teamUtil.angle;
import org.firstinspires.ftc.teamcode.teamUtil.log;
import org.firstinspires.ftc.teamcode.teamUtil.pose2D;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.coordinate2D;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryAssembly;

public class swerveDriveBase {

    static robotConfig r;

    static public swerveModule left;
    static public swerveModule right;

    public static pose2D leftPose2D;
    public static pose2D rightPose2D;

    public static pose2D targetPose2D;
    public static double targetVelocity;
    public static double turnPower;

    public static angle manualTargetHeading;
    double robotHeadingError;
    boolean robotFlipped;
    public static log log;

    static public robotConstants.enabledModules enabledModules;

    public swerveDriveBase(robotConfig r, robotConstants.enabledModules enabledModules){
        swerveDriveBase.enabledModules = enabledModules;
        swerveDriveBase.r = r;

        switch (enabledModules) {
            case LEFT:
                left = new swerveModule(r, robotConstants.moduleSides.LEFT);
                leftPose2D = new pose2D( new coordinate2D(0,0), new angle(0, angle.angleType.ABSOLUTE));
                break;

            case RIGHT:
                right = new swerveModule(r, robotConstants.moduleSides.RIGHT);
                rightPose2D = new pose2D( new coordinate2D(0,0), new angle(0, angle.angleType.ABSOLUTE));
                break;

            case BOTH:
                left = new swerveModule(r, robotConstants.moduleSides.LEFT);
                right = new swerveModule(r, robotConstants.moduleSides.RIGHT);
                leftPose2D = new pose2D( new coordinate2D(-robotConstants.swerveDistance/2,0), new angle(90, angle.angleType.ABSOLUTE));
                rightPose2D = new pose2D( new coordinate2D(robotConstants.swerveDistance/2,0), new angle(90, angle.angleType.ABSOLUTE));
                break;
        }

        manualTargetHeading = new angle(90, angle.angleType.ABSOLUTE);
        robotHeadingError = 0;
        robotFlipped = false;

        targetPose2D = new pose2D(new coordinate2D(0, 0), new angle(0));

        log = new log("SWERVE DRIVE BASE", "target x", "target y", "current x", "current y", "target velocity", "velocity");
    }

    public static void readEncoder(){
        switch (enabledModules){
            case LEFT:
                left.readEncoders();
                leftPose2D.coordinate2D.vector2DUpdate(left.getInstanceDistance(), left.getCurrentModuleAngle());
                break;

            case RIGHT:
                right.readEncoders();
                rightPose2D.coordinate2D.vector2DUpdate(right.getInstanceDistance(), right.getCurrentModuleAngle());
                break;

            case BOTH:
                left.readEncoders();
                right.readEncoders();
                leftPose2D.coordinate2D.vector2DUpdate(left.getInstanceDistance(), left.getCurrentModuleAngle());
                rightPose2D.coordinate2D.vector2DUpdate(right.getInstanceDistance(), right.getCurrentModuleAngle());
                break;
        }
        robotConfig.previousRobotPose2D = robotConfig.robotPose2D;
        robotConfig.robotPose2D = pose2D.moduleProcessor(leftPose2D, rightPose2D, enabledModules);

        log.logData(0, targetPose2D.coordinate2D.x);
        log.logData(1, targetPose2D.coordinate2D.y);
        log.logData(2, robotConfig.robotPose2D.coordinate2D.x);
        log.logData(3, robotConfig.robotPose2D.coordinate2D.y);
        log.logData(4, targetVelocity);
        log.logData(5, "waiting for implementation");
        log.updateLoop(true);

    }

    public static void resetEncoders(){
        switch (enabledModules){
            case LEFT:
                left.resetEncoders();
                break;
            case RIGHT:
                right.resetEncoders();
                break;
            case BOTH:
                left.resetEncoders();
                right.resetEncoders();
                break;
        }
    }

    public void manualDrive(double planarY, double planarX, double headingX, double headingY, double throttle, boolean holdPos){
        manualTargetHeading = angle.atanHandler(headingX, headingY);
        double turnPower = Math.sqrt((headingX*headingX)+(headingY*headingY));

        if(enabledModules== robotConstants.enabledModules.BOTH) {
            setRobotFlippedPlusRobotHeadingError();
        }
        else{
            robotFlipped = false;
            robotHeadingError = 0;
        }

        switch (enabledModules){
            case LEFT:
                left.manualDrive(planarY, planarX, turnPower, robotHeadingError, throttle, holdPos);
                break;

            case RIGHT:
                right.manualDrive(planarY, planarX, turnPower, robotHeadingError, throttle, holdPos);
                break;

            case BOTH:
                left.manualDrive(planarY, planarX, turnPower, robotHeadingError, throttle, holdPos);
                right.manualDrive(planarY, planarX, turnPower, robotHeadingError, throttle, holdPos);
                break;
        }
    }

    public void trajectoryFollowerUpdate(){
        if(!(enabledModules == robotConstants.enabledModules.BOTH)) {
            return;
        }
        robotConfig.currentTrajectoryAssembly.update();
        double xPositionError = targetPose2D.coordinate2D.x - robotConfig.robotPose2D.coordinate2D.x;
        double yPositionError = targetPose2D.coordinate2D.y - robotConfig.robotPose2D.coordinate2D.y;

        setRobotFlippedPlusRobotHeadingError();
        left.ffDrive(turnPower, robotHeadingError, xPositionError, yPositionError, targetVelocity, robotConstants.maxSwerveVelocity);
        right.ffDrive(turnPower, robotHeadingError, xPositionError, yPositionError, targetVelocity, robotConstants.maxSwerveVelocity);
    }

    public void setTrajectoryAssembly(trajectoryAssembly trajectoryAssembly){
        robotConfig.currentTrajectoryAssembly = trajectoryAssembly;
    }

    public void setStartPose2D(pose2D startPose2D){
        robotConfig.robotPose2D = startPose2D;
        robotConfig.previousRobotPose2D = startPose2D;
    }

    private void setRobotFlippedPlusRobotHeadingError(){
        if (Math.abs(robotHeadingError)>90 && robotConstants.robotFlipping){
            robotFlipped = !robotFlipped;
        }
        else{
            robotFlipped = false;
        }
        if (!robotFlipped) {
            robotHeadingError = manualTargetHeading.angleShortDifference(new angle(robotConfig.robotPose2D.angle.value, angle.angleType.ABSOLUTE));

        }
        else{
            robotHeadingError = manualTargetHeading.angleShortDifference(new angle(robotConfig.robotPose2D.angle.value+180, angle.angleType.ABSOLUTE));
        }
        if(isNaN(robotHeadingError)){
            robotHeadingError=0;
        }
    }
}