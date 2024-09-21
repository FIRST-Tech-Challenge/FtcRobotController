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

@Config
public class Robot {
    private IMU imu;
    private double TargetAngle = 0;
    private double RobotAngle = 0;
    private double drive;
    private double slide;
    private double turn;

    public MecanumDriveTrain driveTrain;
    private double RobotX, RobotY;
    private ElapsedTime runtime = new ElapsedTime();

    FtcDashboard dashboard;
    private double robotHeading;
    private double lastTimeAngle;
    private boolean CurrentAlign = true;
    private boolean DriverOrientationDriveMode = true;

    public double derivativeConstantAngle;
    public double proportionalConstantAngle;

    public Telemetry telemetry = null;

    private boolean resettingImu = false;
    private double AutoStartAngle = 0;

    public void init(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap);
    }

    public void IMU_Update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if (orientation.getRoll(AngleUnit.DEGREES) == 0 && orientation.getPitch(AngleUnit.DEGREES) == 0
                && orientation.getYaw(AngleUnit.DEGREES) == 0) {
            if (!resettingImu) {
                telemetry.addData("IMU failed?", "Re-initializing!");
                resettingImu = true;
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
                imu.initialize(new IMU.Parameters(orientationOnRobot));
            }
        } else {
            resettingImu = false;
        }
        telemetry.addData("resettingIMU", resettingImu);
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        robotHeading = orientation.getYaw(AngleUnit.RADIANS);
        RobotAngle = orientation.getYaw(AngleUnit.RADIANS);
        RobotAngle += AutoStartAngle;
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", RobotAngle);
    }

    public void IMUReset() {
        telemetry.addData("Yaw", "Reset" + "ing\n");
        imu.resetYaw();
        TargetAngle = 0;
    }

    private double lastErrorAngle;
    private boolean IsProgramAutonomous;

    public void ProportionalFeedbackControl() {
        double currentTime = runtime.time();
        double derivativeAngle;
        double error = 0;
        if (resettingImu)
            return;
        telemetry.addData("target", TargetAngle);
        error = Wrap((TargetAngle/180)*Math.PI - RobotAngle)*180/Math.PI;
        derivativeAngle = (error - lastErrorAngle)/(currentTime - lastTimeAngle);

        TargetAngle = (RobotAngle * 180 / Math.PI);

        telemetry.addData("ERROR", error);
        telemetry.addData("BEFORE", turn);
        turn -= error * proportionalConstantAngle + (derivativeConstantAngle * derivativeAngle);
        telemetry.addData("AFTER", turn);
        lastTimeAngle = currentTime;
        lastErrorAngle = error;
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