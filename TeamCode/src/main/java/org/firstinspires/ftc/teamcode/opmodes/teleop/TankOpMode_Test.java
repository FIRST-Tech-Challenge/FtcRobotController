package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotWithSpinner;
import org.firstinspires.ftc.teamcode.utils.Tank;
import org.firstinspires.ftc.teamcode.utils.TankRobot;

@TeleOp(name="Basic OpMode with spinner attachment", group="Special Hardware")
public class TankOpMode_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TankRobot tank;

    @Override
    public void runOpMode() {

//        tank = new TankRobot(gamepad1, gamepad2, hardwareMap, telemetry);

        while (opModeIsActive()) {

            tank.main();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", tank.drivetrain.ld1.getPower(), tank.drivetrain.rd1.getPower());
            telemetry.update();
        }
    }
}