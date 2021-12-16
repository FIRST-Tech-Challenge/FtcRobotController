package ca.webber.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ca.webber.ftc.helpers.TeleDriveController;
import ca.webber.ftc.helpers.IntakeController;
import ca.webber.ftc.helpers.TelePivotController;

@TeleOp(name="TeleOp Mode 01")
public class TeleOp_01 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final double SLOW = 20;

    private TeleDriveController teleDriveController;
    private IntakeController intakeController;
    private TelePivotController telePivotController;
    private TouchSensor touchSensor;

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // instantiates the driveController
        teleDriveController = new TeleDriveController(
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "leftBack"),
                hardwareMap.get(DcMotor.class, "rightBack")
        );

        // instantiates the intakeController
        intakeController = new IntakeController(
                hardwareMap.get(DcMotor.class, "leftIntake"),
                hardwareMap.get(DcMotor.class, "rightIntake")
        );

        // instantiates the pivotController
        telePivotController = new TelePivotController(
                hardwareMap.get(DcMotor.class, "pivot")
        );

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // operates the driveController
            if (!gamepad1.left_bumper) {
                teleDriveController.update(
                        (gamepad1.right_bumper ? -1 : 1) * gamepad1.left_stick_x / SLOW,
                        (gamepad1.right_bumper ? -1 : 1) * gamepad1.left_stick_y / SLOW,
                        Math.cbrt(gamepad1.right_stick_x / (2 * SLOW))
                );
            } else {
                teleDriveController.update(
                        (gamepad1.right_bumper ? -1 : 1) * gamepad1.left_stick_x,
                        (gamepad1.right_bumper ? -1 : 1) * gamepad1.left_stick_y,
                        Math.cbrt(gamepad1.right_stick_x / 2)
                );
            }

            // operates the intake controller
            if (gamepad1.dpad_up) {
                intakeController.update(1);
            } else if (gamepad1.dpad_down) {
                intakeController.update(-1);
            } else {
                intakeController.update(0);
            }

            // operates the pivot controller
            telePivotController.update(gamepad1.right_stick_y);
            if (touchSensor.isPressed()) {
                telePivotController.updatePositions();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

}
