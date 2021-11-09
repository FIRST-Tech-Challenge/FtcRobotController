package ca.webber.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ca.webber.ftc.helpers.DriveController;

@TeleOp(name="TeleOp Mode 01", group="Linear Opmode")
public class TeleOp_01 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DriveController driveController;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // instantiates the driveController
        driveController = new DriveController(
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "leftBack"),
                hardwareMap.get(DcMotor.class, "rightBack")
        );

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // operates the driveController
            driveController.update(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y, Math.cbrt(gamepad1.right_stick_x));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

}
