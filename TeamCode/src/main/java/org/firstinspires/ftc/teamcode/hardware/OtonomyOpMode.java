package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RobotDriveOpMode", group = "Linear Opmode")
public class OtonomyOpMode extends LinearOpMode {
    double xAxis;
    double yAxis;
    double rot;

    private ElapsedTime runtime = new ElapsedTime();
  //  private Elevator elevator = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        elevator = new Elevator(hardwareMap, telemetry);
        TevelRobot tevelRobot = new TevelRobot(this);
        double power;
        boolean bumper = false;
//        boolean leftBumper = false;
//        boolean rightBumper = false;
        boolean yButton = false;
       tevelRobot.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        Wheels w = new Wheels(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            xAxis = gamepad1.left_stick_x;
            yAxis = -gamepad1.left_stick_y;
            rot = gamepad1.right_stick_x;
            tevelRobot.getWheels().driveByJoystick(0.5, 0, 0);
        }
    }}

