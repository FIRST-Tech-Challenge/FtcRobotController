package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PatrickTeleOp")
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.init(hardwareMap, true, true, false);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gamepads initialization
            double leftStickY1 = -gamepad1.left_stick_y; // Forward/Backward
            double leftStickX1 = gamepad1.left_stick_x * 1.1; // Strafe (with adjustment)
            double rightStickX1 = gamepad1.right_stick_x; // Rotation
            boolean leftBumper = gamepad1.left_bumper;
            // Speed control
            double speedControl = leftBumper ? 0.3 : 1.0;
            leftStickY1 *= speedControl;
            leftStickX1 *= speedControl;
            rightStickX1 *= speedControl;

            // Telemetry

            robot.move(leftStickX1, leftStickY1, rightStickX1);

            telemetry.addData("X: ", robot.getX());
            telemetry.addData("Y: ", robot.getY());
            telemetry.addData("Angle: ", robot.getAngle());
            telemetry.update();
        }


    }



}
