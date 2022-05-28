package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "BlueLeftTeleopRegionals")
//@Disabled

public class BlueLeftTeleOp extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, true ,false, 0);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {
            robot.teleopLoop(1,35.5,0);
        }

        idle();
    }
}