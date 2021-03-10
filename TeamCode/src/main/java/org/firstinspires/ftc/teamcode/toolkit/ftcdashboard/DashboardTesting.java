package org.firstinspires.ftc.teamcode.toolkit.ftcdashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.SERVO_VAL;

@Autonomous
public class DashboardTesting extends UpliftAuto {
    UpliftRobot robot;
    DriveSubsystem drive;
    WobbleSubsystem wobble;
    FtcDashboard dashboard;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        drive = robot.driveSub;
        wobble = robot.wobbleSub;
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void initAction() {}

    @Override
    public void body() throws InterruptedException {
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Servo Val", SERVO_VAL);
        dashboardTelemetry.update();
        wobble.closeWobble();
        wobble.highWobble();
        sleep(5000);
    }

    @Override
    public void exit() {
        robot.stopThreads();
    }

}