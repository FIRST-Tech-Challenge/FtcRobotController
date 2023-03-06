package org.firstinspires.ftc.teamcode.subsystems.swerve;

import static java.lang.Double.isNaN;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamUtil.Angle;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.Log;
import org.firstinspires.ftc.teamcode.teamUtil.Pose2D;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.Coordinate2D;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.TrajectoryAssembly;

public class SwerveDriveBase extends Subsystem {

    RobotConfig r;

    static public SwerveModule left;
    static public SwerveModule right;

    public static IMU imu;

    public static Pose2D leftPose2D;
    public static Pose2D rightPose2D;

    public static Pose2D targetPose2D;
    public static double targetVelocity;
    public static double turnPower;

    public static Angle manualTargetHeading;
    double robotHeadingError;
    boolean robotFlipped;
    public static Log log;

    public static RobotConstants.enabledModules enabledModules;

    public SwerveDriveBase(RobotConfig r, RobotConstants.enabledModules enabledModules) {
        SwerveDriveBase.enabledModules = enabledModules;
        this.r = r;
    }
    public SwerveDriveBase(RobotConstants.enabledModules enabledModules){
        SwerveDriveBase.enabledModules = enabledModules;
        r = RobotConfig.getInstance();
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
        manualTargetHeading = Angle.atanHandler(headingX, headingY);
        double turnPower = Math.sqrt((headingX*headingX)+(headingY*headingY));

        if(enabledModules== RobotConstants.enabledModules.BOTH) {
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
        if(!(enabledModules == RobotConstants.enabledModules.BOTH)) {
            return;
        }
        if(RobotConfig.currentTrajectoryAssembly != null){
            RobotConfig.currentTrajectoryAssembly.update();
        }
        double xPositionError = targetPose2D.coordinate2D.x - RobotConfig.robotPose2D.coordinate2D.x;
        double yPositionError = targetPose2D.coordinate2D.y - RobotConfig.robotPose2D.coordinate2D.y;

        setRobotFlippedPlusRobotHeadingError();
        left.ffDrive(turnPower, robotHeadingError, xPositionError, yPositionError, targetVelocity, RobotConstants.maxSwerveVelocity);
        right.ffDrive(turnPower, robotHeadingError, xPositionError, yPositionError, targetVelocity, RobotConstants.maxSwerveVelocity);
    }

    public void setTrajectoryAssembly(TrajectoryAssembly trajectoryAssembly){
        RobotConfig.currentTrajectoryAssembly = trajectoryAssembly;
    }

    public void setStartPose2D(Pose2D startPose2D){
        RobotConfig.robotPose2D = startPose2D;
        RobotConfig.previousRobotPose2D = startPose2D;
    }

    private void setRobotFlippedPlusRobotHeadingError(){
        if (Math.abs(robotHeadingError)>90 && RobotConstants.robotFlipping){
            robotFlipped = !robotFlipped;
        }
        else{
            robotFlipped = false;
        }
        if (!robotFlipped) {
            robotHeadingError = manualTargetHeading.angleShortDifference(new Angle(RobotConfig.robotPose2D.angle.value, Angle.angleType.ABSOLUTE));

        }
        else{
            robotHeadingError = manualTargetHeading.angleShortDifference(new Angle(RobotConfig.robotPose2D.angle.value+180, Angle.angleType.ABSOLUTE));
        }
        if(isNaN(robotHeadingError)){
            robotHeadingError=0;
        }
    }

    @Override
    public void init() {
        imu = r.opMode.hardwareMap.get(IMU.class, ConfigNames.imu);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                new Orientation(
                                        AxesReference.INTRINSIC,
                                        AxesOrder.ZYX,
                                        AngleUnit.DEGREES,
                                        90,0,0, //NO IDEA HOW THESE WORK RN LMAO (kinda)
                                        0 //ignore, something about time or smth
                                )
                        )
                )
        );
        switch (enabledModules) {
            case LEFT:
                left = new SwerveModule(r, RobotConstants.moduleSides.LEFT);
                leftPose2D = new Pose2D( new Coordinate2D(0,0), new Angle(0, Angle.angleType.ABSOLUTE));
                break;

            case RIGHT:
                right = new SwerveModule(r, RobotConstants.moduleSides.RIGHT);
                rightPose2D = new Pose2D( new Coordinate2D(0,0), new Angle(0, Angle.angleType.ABSOLUTE));
                break;

            case BOTH:
                left = new SwerveModule(r, RobotConstants.moduleSides.LEFT);
                right = new SwerveModule(r, RobotConstants.moduleSides.RIGHT);
                leftPose2D = new Pose2D( new Coordinate2D(-RobotConstants.swerveDistance/2,0), new Angle(90, Angle.angleType.ABSOLUTE));
                rightPose2D = new Pose2D( new Coordinate2D(RobotConstants.swerveDistance/2,0), new Angle(90, Angle.angleType.ABSOLUTE));
                break;
        }

        manualTargetHeading = new Angle(90, Angle.angleType.ABSOLUTE);
        robotHeadingError = 0;
        robotFlipped = false;

        targetPose2D = new Pose2D(new Coordinate2D(0, 0), new Angle(0));

        log = new Log("SWERVE_DRIVE_BASE", "target x", "target y", "target heading", "current heading", "current x", "current y", "target velocity", "velocity");
    }

    @Override
    public void read() {
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
        RobotConfig.previousRobotPose2D = RobotConfig.robotPose2D;
        //the commented out method runs odometry for tracking, the other one does not and is probably doggest of shit maybe maybe maybe, also uses IMU for heading
        //robotConfig.robotPose2D = new pose2D(new coordinate2D(r.odometry.getPoseEstimate().getX(), r.odometry.getPoseEstimate().getY()), new angle(r.odometry.getPoseEstimate().getHeading()));
        RobotConfig.robotPose2D = Pose2D.moduleProcessor(leftPose2D, rightPose2D, new Angle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), enabledModules);

        log.logData(0, targetPose2D.coordinate2D.x);
        log.logData(1, targetPose2D.coordinate2D.y);
        log.logData(2, targetPose2D.angle.value);

        log.logData(3, RobotConfig.robotPose2D.angle.value);
        log.logData(4, RobotConfig.robotPose2D.coordinate2D.x);
        log.logData(5, RobotConfig.robotPose2D.coordinate2D.y);

        log.logData(6, targetVelocity);
        log.logData(7, "waiting for implementation");
        log.updateLoop(true);
    }

    @Override
    public void update() {

    }

    @Override
    public void close() {
        log.close();
        switch (enabledModules){
            case LEFT:
                left.log.close();
                break;
            case RIGHT:
                right.log.close();
                break;
            case BOTH:
                left.log.close();
                right.log.close();
                break;
        }
    }
}