package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

@Autonomous(name = "Shooter Velocity Tester", group = "Hardware Testers")
public class ShooterVelocityTester extends UpliftAuto {

    UpliftRobot robot;
    VelocityData velocityData;
    FtcDashboard ftcDashboard;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        this.velocityData = robot.velocityData;
    }

    @Override
    public void initAction() {
        ftcDashboard = FtcDashboard.getInstance();
    }

    @Override
    public void body() throws InterruptedException {
        waitForStart();
        
        while(opModeIsActive()) {
            robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(DashboardConstants.Kp, DashboardConstants.Ki, DashboardConstants.Kd, DashboardConstants.Kf));
            robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(DashboardConstants.Kp, DashboardConstants.Ki, DashboardConstants.Kd, DashboardConstants.Kf));
            robot.shooter1.setVelocity(DashboardConstants.targetVel);
            robot.shooter2.setVelocity(DashboardConstants.targetVel);
            Log.d("Shooter1 Velocity", robot.shooter1Vel + "");
            Log.d("Shooter2 Velocity", robot.shooter2Vel + "");
        }

        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);

    }

    @Override
    public void exit() {
        robot.stopThreads();
    }
}
