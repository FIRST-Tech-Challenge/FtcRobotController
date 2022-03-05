package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "Turret_Testing")
//@Disabled

public class Turret_Testing extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false ,false);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        double diSTANCe=0;

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {

            diSTANCe+=gamepad2.left_stick_y/10;
            robot.TurretAngleControlRotating(diSTANCe);
//            sleep(100);
            telemetry.addData("diSTANCe",diSTANCe);
            telemetry.addData("runtime",getRuntime());
            telemetry.update();

            robot.TurretManualRotation(gamepad2.right_stick_x);

//            sleep(100);

            robot.TurretManualExtension(gamepad2.right_trigger, gamepad2.left_trigger);

            telemetry.addData("extension pos", robot.Turret_Extension_Position());
            telemetry.addData("rotation pos", robot.Turret_Rotation_Position());

//            sleep(100);
        }

        idle();
    }
}