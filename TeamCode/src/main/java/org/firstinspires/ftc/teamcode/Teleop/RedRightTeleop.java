package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;


@TeleOp(name = "RedRightTeleopRegionals")
//@Disabled
public class RedRightTeleop extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true ,false,90);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        robot.tseToPosition(0.8);

        while (!isStopRequested()) {
            robot.teleopLoop(-1,0,0);
        }


        idle();
    }
}