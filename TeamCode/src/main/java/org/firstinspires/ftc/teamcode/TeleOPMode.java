package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        boolean mat = false;
        boolean claw = false;
        boolean sideArm = false; //Outside of loop()
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.color.blue);

        waitForStart();

        while (opModeIsActive()) {
            //Gamepad 1
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            //Using a cube to add exponential growth to the control of rotation
            double rotate = gamepad1.right_stick_x * robot.motorPower;


            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 0.65;
            //Wheel control
            robot.mixDrive(forward, strafe, rotate);

            //Front servo control
            if (gamepad1.x && !mat){
                if (robot.MatServos.getPosition() == 0) robot.MatServos.setPosition((float)90 / 280);
                else robot.MatServos.setPosition(0);
                mat = true;
            } else if(!gamepad1.x) mat = false;

            //SideArm control
            if(gamepad1.b && !sideArm) {
                if(robot.SideArm.getPosition() == 0) robot.SideArm.setPosition((float) 110 / 180);
                else robot.SideArm.setPosition(0);
                sideArm = true;
            } else if(!gamepad1.b) sideArm = false;

            if (gamepad1.y) robot.SetSideArm(70, 180);



            //Gamepad 2
            if (gamepad2.left_trigger > 0.7) robot.liftPower = 0.2;
            else robot.liftPower = 0.4;

            robot.armLift.setPower(gamepad2.left_stick_y);

            if(gamepad2.right_trigger > 0.7 && !claw) {
                if(robot.BlockGrips.getPosition() == 0) robot.controlClaw(30);
                else robot.controlClaw(0);
                claw = true;
            } else if(gamepad2.right_trigger < 0.7) claw = false;

            telemetry.addData("Red: ", robot.colorSensor.red());
            telemetry.addData("Green: ", robot.colorSensor.green());
            telemetry.addData("Blue: ", robot.colorSensor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        }

    }
