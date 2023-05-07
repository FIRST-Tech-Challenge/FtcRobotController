package org.firstinspires.ftc.teamcode.drivetests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MoveElevator;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

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
    double turnIdle = 0.1;

    public static double turnStick = 0;

    FtcDashboard dashboard = null;
    RobotHardware robot = new RobotHardware();
    Drivetrain drivetrain = null;
    MoveElevator elevator = null;

    @Override
    public void go() {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        //MoveElevator moveElevator = new MoveElevator();

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);

        robot.init(this);

        waitForStart();

        Scheduler.launchOnStart(this, () -> {

            drivetrain.robotGoSkrtSkrt(driver.left_stick_x.get() * strafeSpeed, -driver.left_stick_y.get() * forwardSpeed, robot.getTurnAmount(driver.right_stick_x.get() * turnSpeed));

            driver.a.onRise(() -> robot.setFieldCentric(true));
            driver.b.onRise(() -> robot.setFieldCentric(false));

            driver.left_bumper.onRise(() -> robot.setTotalSpeed(0.25))
                              .onFall(() -> robot.setTotalSpeed(0.5));

            driver.right_bumper.onRise(() -> robot.setTotalSpeed(1))
                               .onFall(() -> robot.setTotalSpeed(0.5));

            driver.dpad_up.onRise(() -> robot.setElevatorPosition(100));
            driver.dpad_down.onRise(() -> robot.setElevatorPosition(0));

            robot.updateElevatorPosition();

            turnStick = driver.right_stick_x.get();

            mTelemetry().addData("Field Centric:", robot.isFieldCentric());

            mTelemetry().addData("turning", robot.getTurnAmount(driver.right_stick_x.get() * turnSpeed));
            mTelemetry().addData("Heading:", robot.getHeading());
            mTelemetry().addData("Target Heading", Math.toDegrees(robot.getAngleTarget()));
            mTelemetry().addData("Right Stick X", turnStick * 180);
            mTelemetry().addData("Coming To Rest", robot.isInComingToRest());

            mTelemetry().addData("Ele Power", robot.getElePower());
            mTelemetry().update();

        });
    }
}
