package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.BasicChassis;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot;


@TeleOp(name = "RedLeftTeleopRegionals")
@Disabled

public class RedLeftTeleOp extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true ,false,0);
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
            robot.teleopLoop(-1,-35.5,0);
        }


        idle();
    }
}