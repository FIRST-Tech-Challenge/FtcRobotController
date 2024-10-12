package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;
import android.view.View;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.resourses.Odometry;

@Config
public class Robot {

    private double TargetAngle = 0;
    private double lastTimeAngle;

    private double drive;
    private double slide;
    private double turn;

    public MecanumDriveTrain driveTrain;
    public Odometry odometry;
    private double RobotX, RobotY;
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard;
    private boolean CurrentAlign = true;
    private boolean DriverOrientationDriveMode = true;
    
    public double derivativeConstantAngle;
    public double proportionalConstantAngle;
    public Telemetry telemetry = null;
    
    private double lastErrorAngle;
    private boolean IsProgramAutonomous;
    
    public void init(HardwareMap hardwareMap, double robotX, double robotY, double robotAngle){
        driveTrain = new MecanumDriveTrain(hardwareMap);
        odometry = new Odometry(robotX,robotY,robotAngle, telemetry);
    }

    public void ProportionalFeedbackControl() {
        double currentTime = runtime.time();
        double derivativeAngle;
        double error = 0;
        if (odometry.isResetingIMU())
            return;
        telemetry.addData("target", TargetAngle);
        error = Wrap((TargetAngle/180)*Math.PI - odometry.getRobotAngle())*180/Math.PI;
        derivativeAngle = (error - lastErrorAngle)/(currentTime - lastTimeAngle);

        TargetAngle = (odometry.getRobotAngle() * 180 / Math.PI);

        telemetry.addData("ERROR", error);
        telemetry.addData("BEFORE", turn);
        turn -= error * proportionalConstantAngle + (derivativeConstantAngle * derivativeAngle);
        telemetry.addData("AFTER", turn);
        lastTimeAngle = currentTime;
        lastErrorAngle = error;
    }
    public void IMUReset() {
        TargetAngle = 0;
        odometry.IMUReset();
    }
    
    double Wrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}