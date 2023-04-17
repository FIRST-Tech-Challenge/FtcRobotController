package org.firstinspires.ftc.teamcode.drivetests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.PIDOpenClosed;

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

    public static double Kp = 2.5;
    public static double Ki = 0.4;
    public static double Kd = 0.4;
    public static double integralSumMax = 0.25;
    public static double stabilityThresh = 2;
    public static double lowPassGain = 0.5;

    public static double turnStick = 0;
    FtcDashboard dashboard = null;

    RobotHardware robot = new RobotHardware();

    @Override
    public void go() {

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        robot.init(this);

        waitForStart();

        Scheduler.launchOnStart(this, () -> {

            robot.robotGoSkrtSkrt(driver.left_stick_x.get() * strafeSpeed, driver.left_stick_y.get() * forwardSpeed, robot.getTurnAmount(driver.right_stick_x.get() * turnSpeed));

            robot.turningPIDSetter(Kp, Ki, Kd, integralSumMax, stabilityThresh, lowPassGain);

            driver.a.onRise(() -> robot.setFieldCentric(true));
            driver.b.onRise(() -> robot.setFieldCentric(false));
            driver.x.onRise(() -> robot.setAngleTarget(90));
            driver.y.onRise(() -> robot.setAngleTarget(0));

            turnStick = driver.right_stick_x.get();

            telemetry.addData("Field Centric:", robot.isFieldCentric());

            telemetry.addData("turning", robot.getTurnAmount(driver.right_stick_x.get() * turnSpeed));
            telemetry.addData("Heading:", robot.getHeading());
            telemetry.addData("Target Heading", Math.toDegrees(robot.getAngleTarget()));
            telemetry.addData("Right Stick X", turnStick * 180);
            telemetry.addData("LeftTog", robot.isTurnLeftTog());
            telemetry.addData("RightTog", robot.isTurnRightTog());
            telemetry.update();

        });
    }
}
