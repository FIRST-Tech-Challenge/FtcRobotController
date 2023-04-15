package org.firstinspires.ftc.teamcode.drivetests;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Config
@TeleOp(name="Drive Test", group="Linear Opmode")
public class MecanumTesting extends BlackOp {

    //DRIVE VARIABLES
    double strafeSpeed = 1;
    double forwardSpeed = 1;
    double turnSpeed = 1;

    RobotHardware robot = new RobotHardware();

    @Override
    public void go() {

        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        robot.init(this);

        waitForStart();

        Scheduler.launchOnStart(this, () -> {


            robot.robotGoSkrtSkrt(driver.left_stick_x.get() * strafeSpeed, driver.left_stick_y.get() * forwardSpeed, robot.getTurnAmount(driver.right_stick_x.get() * turnSpeed));

            driver.a.onRise(() -> robot.setFieldCentric(true));
            driver.b.onRise(() -> robot.setFieldCentric(false));

            telemetry.addData("Field Centric:", robot.isFieldCentric());
            telemetry.addData("Heading:", robot.getHeading());
            telemetry.update();

        });
    }
}
