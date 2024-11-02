package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name ="ClimbTester")
public class ClimbTester extends LinearOpMode {
    private Climb climb;
    private GamepadEvents controller;
    private MechDrive robot;
    public void runOpMode() throws InterruptedException {


        climb = new Climb(hardwareMap, "climb");
        controller = new GamepadEvents(gamepad1);
        robot = new MechDrive(hardwareMap);

        waitForStart();
        while(opModeIsActive())
        {
            double climbPower = controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue();
            climb.moveClimb(climbPower);

            double forward = -controller.left_stick_y;
            double strafe = -controller.left_stick_x;
            double rotate = -controller.right_stick_x;

            robot.drive(forward, strafe, rotate);

            telemetry.addData("Climb Power", climbPower);
            telemetry.update();
            controller.update();
        }
    }
}
