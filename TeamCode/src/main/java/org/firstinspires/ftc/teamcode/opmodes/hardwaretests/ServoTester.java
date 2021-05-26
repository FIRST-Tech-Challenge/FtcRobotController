package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.FLICKER_POS;

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
//            robot.intakeLifter.setPosition(INTAKE_LIFTER_POS);
            robot.flicker.setPosition(FLICKER_POS);
//            robot.wobbleSub.setWobblePosition(WOBBLE_POS);
//            robot.clamp.setPosition(CLAMP_POS);
            dashTelem.addData("Potentiometer Val:", robot.potentiometer.getVoltage());
        }

    }

    @Override
    public void exit() {
        robot.stopThreads();
    }
}

