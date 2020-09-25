package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);

        waitForStart();

        while (opModeIsActive()) {
            //Gamepad 1  **Drivetrain**
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            //Using a cube to add exponential growth to the control of rotation
            double rotate = gamepad1.right_stick_x * robot.motorPower;


            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 0.65;
            //Wheel control
            robot.mixDrive(forward, strafe, rotate);

            //Gamepad 2  **Gun and intake**


            telemetry.addData("Red: ", robot.colorSensor.red());
            telemetry.addData("Green: ", robot.colorSensor.green());
            telemetry.addData("Blue: ", robot.colorSensor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        }

    }
