package org.firstinspires.ftc.teamcode.toolkit.background;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;

public class VelocityData extends Background {

    UpliftRobot robot;
    LinearOpMode opMode;

    FtcDashboard dashboard;
    Telemetry dashTelem;

    private static final int TICKS_PER_SHOOTER_WHEEL_ROTATION = 28;

    public VelocityData(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;

        // COMMENT OUT BEFORE COMP
        this.dashboard = FtcDashboard.getInstance();
        this.dashTelem = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        robot.shooter1Vel = robot.shooter1.getVelocity();
        robot.shooter2Vel = robot.shooter2.getVelocity();
//            Log.i("Shooter1Velocity", robot.shooter1SmoothVel + "");
//            Log.i("Shooter2Velocity", robot.shooter2SmoothVel + "");
        dashTelem.addData("Shooter 1 Velocity", robot.shooter1Vel + "");
        dashTelem.addData("Shooter 2 Velocity", robot.shooter2Vel + "");
        dashTelem.addData("High Goal Velocity", robot.highGoalVelocity);
        dashTelem.addData("Powershot Velocity", robot.powerShotVelocity);
        dashTelem.addData("Lower Limit", 1000);
        dashTelem.addData("Upper Limit", 3000);
//        dashTelem.update();

    }

    public boolean isAutoHighGoalReady() {
        double lowerLimit = robot.autoHighGoalVelocity - 50;
        double upperLimit = robot.autoHighGoalVelocity + 50;
        return (robot.shooter1Vel > lowerLimit && robot.shooter2Vel > lowerLimit && robot.shooter1Vel < upperLimit && robot.shooter2Vel < upperLimit);
    }

    public boolean isHighGoalShooterReady() {
        double lowerLimit = robot.highGoalVelocity - 50;
        double upperLimit = robot.highGoalVelocity + 50;
        return (robot.shooter1Vel > lowerLimit && robot.shooter2Vel > lowerLimit && robot.shooter1Vel < upperLimit && robot.shooter2Vel < upperLimit);
    }

    public boolean isPowerShotShooterReady() {
        double lowerLimit = robot.powerShotVelocity - 50;
        double upperLimit = robot.powerShotVelocity + 50;
        return (robot.shooter1Vel > lowerLimit && robot.shooter2Vel > lowerLimit && robot.shooter1Vel < upperLimit && robot.shooter2Vel < upperLimit);
    }
}
