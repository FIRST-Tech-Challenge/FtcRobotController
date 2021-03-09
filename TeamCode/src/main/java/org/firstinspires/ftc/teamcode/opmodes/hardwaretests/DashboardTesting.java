package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.FlickerCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.commands.WobbleCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

import static org.firstinspires.ftc.teamcode.opmodes.hardwaretests.DashboardConstants.SERVO_VAL;

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