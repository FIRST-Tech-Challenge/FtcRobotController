package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DemoTeleOp")
public class DemoTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.init(hardwareMap, true, false, false);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gamepads initialization
            double leftStickY1 = -gamepad1.left_stick_y; // Forward/Backward
            double leftStickX1 = gamepad1.left_stick_x * 1.1; // Strafe (with adjustment)
            double rightStickX1 = gamepad1.right_stick_x; // Rotation

            // Speed control
            double speedControl = 0.8;
            leftStickY1 *= speedControl;
            leftStickX1 *= speedControl;
            rightStickX1 *= speedControl;


            robot.move(leftStickX1, leftStickY1, rightStickX1);

        }


    }



}
