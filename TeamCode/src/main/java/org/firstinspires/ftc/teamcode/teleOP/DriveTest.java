package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode {
    GamepadEvents controller1;
    DriveTrain robot;
    @Override
    public void runOpMode() throws InterruptedException
    {
        controller1 = new GamepadEvents(gamepad1);
        robot = new DriveTrain(hardwareMap,"frontLeft", "backLeft", "frontRight",
                "backRight");

        waitForStart();
        while(opModeIsActive())
        {
            robot.cubedDrive(-controller1.left_stick_y ,controller1.left_stick_x, controller1.right_stick_x);
            telemetry.addLine("On");

            controller1.update();
            telemetry.update();

        }
    }
}
