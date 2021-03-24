package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.CLAMP_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.FLICKER_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.INTAKE_LIFTER_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.STICK_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.SWEEPER_JOINT_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.SWEEPER_LEFT_POWER;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.SWEEPER_RIGHT_POWER;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.WOBBLE_POS;

@Autonomous(name = "Servo Tester", group = "Hardware Testers")
public class ServoTester extends UpliftAuto {

    UpliftRobot robot;
    FtcDashboard ftcDashboard;
    Telemetry dashTelem;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {
        ftcDashboard = FtcDashboard.getInstance();
        dashTelem = ftcDashboard.getTelemetry();
    }

    @Override
    public void body() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()) {
//            robot.sweeperJoint.setPosition(SWEEPER_JOINT_POS);
//            robot.sweeperRight.setPower(SWEEPER_RIGHT_POWER);
//            robot.sweeperLeft.setPower(SWEEPER_LEFT_POWER);
//            robot.stick.setPosition(STICK_POS);
            robot.intakeLifter.setPosition(INTAKE_LIFTER_POS);

//            robot.flicker.setPosition(FLICKER_POS);
//            robot.wobbleSub.setWobblePosition(WOBBLE_POS);
//            robot.clamp.setPosition(CLAMP_POS);
        }

    }

    @Override
    public void exit() {
        robot.stopThreads();
    }
}

