package ca.webber.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ca.webber.ftc.helpers.TeleBucketRotateController;
import ca.webber.ftc.helpers.TeleCarouselController;
import ca.webber.ftc.helpers.TeleCloserController;
import ca.webber.ftc.helpers.TeleDriveController;
import ca.webber.ftc.helpers.IntakeController;
import ca.webber.ftc.helpers.TelePivotController;
import ca.webber.ftc.helpers.TeleElevatorController;

@TeleOp(name="TeleOp Mode 01")
public class TeleOp_01 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final double SLOW = 20;

    private TeleDriveController teleDriveController;
    private IntakeController intakeController;
    private TelePivotController telePivotController;
    private TeleElevatorController teleElevatorController;
    private TeleCloserController teleCloserController;
    private TeleCloserController teleBucketRotateController;
    private TeleCarouselController teleCarouselController;
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
                hardwareMap.get(Servo.class, "pivot")
        );

        // instantiates the elevatorController
        teleElevatorController = new TeleElevatorController(
                hardwareMap.get(DcMotor.class, "elevator")
        );

        // Servos
        teleCloserController = new TeleCloserController(
                hardwareMap.get(Servo.class, "closer")
        );

        teleBucketRotateController = new TeleCloserController(
                hardwareMap.get(Servo.class, "bucketRotate")
        );
        teleBucketRotateController.toggle();

        teleCarouselController = new TeleCarouselController(
                hardwareMap.get(DcMotor.class, "carousel")
        );

        // touch controller
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // operates the driveController
            if (gamepad1.left_bumper) {                                                            // controller 1 left bumper
                teleDriveController.update(
                        (gamepad1.right_bumper ? 1 : -1) * gamepad1.left_stick_x * 10 / SLOW,         // controller 1 right bumper
                        (gamepad1.right_bumper ? 1 : -1) * gamepad1.left_stick_y * 10 / SLOW,         // controller 1 left stick x and y
                        Math.cbrt(gamepad1.right_stick_x / (2 * SLOW))                              // controller 1 right stick x
                );
            } else {
                teleDriveController.update(
                        (gamepad1.right_bumper ? 1 : -1) * gamepad1.left_stick_x,
                        (gamepad1.right_bumper ? 1 : -1) * gamepad1.left_stick_y,
                        Math.cbrt(gamepad1.right_stick_x / 2)
                );
            }

            // operates the intake controller
            intakeController.update(0.5 * gamepad2.right_stick_y);                               // controller 2 right stick y

            // operates the elevator
            if (gamepad2.right_trigger > gamepad1.left_trigger) {                                   // controller 2 right trigger
                teleElevatorController.ascend(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > gamepad2.right_trigger) {                              // controller 2 left trigger
                teleElevatorController.descend(gamepad2.left_trigger, touchSensor.isPressed());
            }
            else {
                teleElevatorController.stop();
            }

            // operates the servos
            teleCloserController.update(gamepad2.a);                                                // controller 2 a

            // operates the servos
            teleBucketRotateController.update(gamepad2.x);                                          // controller 2 x

            // operates the pivot controller
            telePivotController.update(gamepad2.left_stick_y);                                      // controller 2 left stick y

            // operates the carousel spinner
            teleCarouselController.update(gamepad2.b);                                              // controller 2 b

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

}
