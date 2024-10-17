package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.resourses.PIDController;
import org.firstinspires.ftc.teamcode.resourses.Odometry;

@Config
public class Robot {
   
    
    //used for how fast the turning input is used.
    public static double maxTurnDegPerSecond = 1000;
    public static double pCon = 0.017;
    public static double dCon = 0;
    
    private double drive;
    private double slide;
    private double turn;
    //TODO Make more permanent system to detect turning
    public boolean turningBoolean;
    public MecanumDriveTrain driveTrain;
    public Odometry odometry;
    private double RobotX, RobotY;
    private ElapsedTime runtime = new ElapsedTime();
    //TODO Tune the pConstant and d Constant numbers, these are place holders.
    public PIDController anglePID = new PIDController(pCon,dCon, runtime);
    
    FtcDashboard dashboard;
    private double robotHeading;
    private double lastTimeAngle;
    private boolean CurrentAlign = true;
    private boolean DriverOrientationDriveMode = true;
    
    public double derivativeConstantAngle;
    public double proportionalConstantAngle;
    public Telemetry telemetry;
    private boolean resettingImu = false;
    private double AutoStartAngle = 0;
    
    private double currentLoopTime, previousLoopTime;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, double x, double y, double angle){
        this.telemetry = telemetry;
        driveTrain = new MecanumDriveTrain(hardwareMap,telemetry);
        odometry = new Odometry(x,y,angle,telemetry,hardwareMap);
    }
    
    public double getDeltaTime(){
        double deltaTime;
        currentLoopTime = runtime.seconds();
        deltaTime = currentLoopTime-previousLoopTime;
        previousLoopTime = currentLoopTime;
        return deltaTime;
    }
    
    public void IMUReset() {
        odometry.IMUReset();
        anglePID.setTarget(0);
    }
    
    private double lastErrorAngle;
    private boolean IsProgramAutonomous;
    
    public double radiansToDegrees(double radians){
        return radians*(180/Math.PI);
    }
    
    public double degreesToRadians(double degrees){
        return degrees*(Math.PI/180);
    }
    
    public void turnUpdate() {
        if (resettingImu){
            return;
        }
        anglePID.setTarget(odometry.getRobotAngle());
        anglePID.update(odometry.getRobotAngle());
        
    }
    

    
    
    
}