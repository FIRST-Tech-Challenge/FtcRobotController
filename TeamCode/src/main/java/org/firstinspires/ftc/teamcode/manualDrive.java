package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class manualDrive extends LinearOpMode{
    @Override
    public void runOpMode() {

        RobotController robotController = new RobotController(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        robotController.cycleController.start();
        robotController.elevatorController.start();
        robotController.driveController.start();

        while (opModeIsActive()){
            gamepad.a = gamepad1.a;
            gamepad.b = gamepad1.b;
            gamepad.y = gamepad1.y;
            gamepad.x = gamepad1.x;

            gamepad.left_trigger = gamepad1.left_trigger;
            gamepad.right_trigger = gamepad1.right_trigger;

            gamepad.left_bumper = gamepad1.left_bumper;
            gamepad.right_bumper = gamepad1.right_bumper;

            gamepad.left_stick_y = gamepad1.left_stick_y;
            gamepad.left_stick_x = gamepad1.left_stick_x;
            gamepad.right_stick_x = gamepad1.right_stick_x;

        }

        robotController.terminate();
        telemetry.clearAll();
        telemetry.addLine("stop");
        telemetry.update();

    }
}

class gamepad{
    public static double right_trigger = 0;
    public static double left_trigger = 0;
    public static double right_stick_x = 0;
    public static double left_stick_x = 0;
    public static double left_stick_y = 0;

    public static boolean right_bumper = false;
    public static boolean left_bumper = false;

    public static boolean a = false;
    public static boolean b = false;
    public static boolean x = false;
    public static boolean y = false;

    public static boolean isElevatorControllerActive = true;
    public static boolean isDriveControllerActive = true;
    public static boolean isCycleControllerActive = true;

    public static boolean isStopRequested = false;
}