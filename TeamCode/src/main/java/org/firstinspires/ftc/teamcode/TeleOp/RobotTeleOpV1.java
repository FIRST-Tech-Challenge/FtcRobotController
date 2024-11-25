package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Config
@TeleOp
public class RobotTeleOpV1 extends OpMode {

    //Declare all of the classes
    public RobotHardware robot;
    public RobotMovement robotMovement;
    public RobotSlides robotSlides;

    public static double motorMaxSpeed = 0.4;

    @Override
    public void init () {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        robotMovement = new RobotMovement(gamepad1, gamepad2, robot, motorMaxSpeed);

        robotSlides = new RobotSlides(gamepad1, gamepad2, robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop () {
        robotMovement.robotDriveTrain();
        //robotSlides.robotSlideControl();
    }
}
