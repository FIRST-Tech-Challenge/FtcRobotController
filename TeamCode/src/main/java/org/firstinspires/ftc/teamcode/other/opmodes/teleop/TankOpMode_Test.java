package org.firstinspires.ftc.teamcode.other.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.utils.TeleOpMovementPlane;

@TeleOp(name="Basic OpMode with Tank Robot", group="Special Hardware")
public class TankOpMode_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TeleOpMovementPlane tank;

    @Override
    public void runOpMode() {

//        tank = new TankRobot(gamepad1, gamepad2, hardwareMap, telemetry);

        while (opModeIsActive()) {

            tank.main();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", tank.getDrivetrain().getLeftTop().getDcMotor().getPower(), tank.getDrivetrain().getLeftTop().getDcMotor().getPower());
            telemetry.update();
        }
    }
}