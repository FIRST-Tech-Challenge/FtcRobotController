package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;

@TeleOp(name="Drive Red", group="Linear Opmode")
public class MainDriveProgramRed extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;
    boolean canToggleHandPosition = true;
    boolean driverControl = true;

    @Override
    public void runOpMode() {

        // create instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            /**
             * Driving
             */

            if (gamepad1.dpad_up) {
                rrDrive.pixelDropRedFar();
            } else if (gamepad1.dpad_down) {
                rrDrive.pixelDropRedNear();
            } else {
                rrDrive.updateMotorsFromStick(gamepad1);
                rrDrive.update();
            }



            /**
             * Intake
             */
            if (gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5){
                intakeSystem.runIntakeSystem(1);
            } else if (gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5){
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                intakeSystem.runIntakeSystem(0);
            }


            /**
             * Climber
             */
            if (gamepad1.right_stick_button) {
            arm.releaseHook();
            }


            /**
             * Arm slides
             */
            if (gamepad2.dpad_up){
                arm.extendAndDrop(LiftArm.Distance.FULL);
                arm.holdArm();
            } else if (gamepad2.dpad_right){
                arm.extendAndDrop(LiftArm.Distance.HALF);
                arm.holdArm();
            } else if (gamepad2.dpad_left) {
                arm.extendAndDrop(LiftArm.Distance.AUTO);
                arm.holdArm();
            } else if (gamepad2.dpad_down) {
                arm.retract();
            }



            /**
             * Trapdoor
             */
            if (gamepad2.right_bumper) {
                arm.openTrapdoor();
            } else {
                arm.closeTrapdoor();
            }



            /**
             * Airplane
             */
            if (gamepad2.x && gamepad2.a) {
                //Release airplane servo
            }


            /**
             * Climber
             */
            if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
                arm.releaseHook();
            } else {
                arm.secureHook();
            }


            /**
             * Slow mode toggle
             */
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;

                //Toggle between slow and normal speeds
                switch (rrDrive.currentSpeed) {
                    case SLOW:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.SLOW);
                        break;
                }
            }


            /**
             * Telemetry data
             */
            telemetry.addData("Trapdoor: ", arm.trapdoor.getPosition());
            telemetry.addData("Hand: ", arm.getHandPosition());
            telemetry.addData("Intake: ", arm.intakePower);
            telemetry.addData("Slides (R, L): ", arm.getSlidePosition() + ", " + arm.getSlidePosition());

            telemetry.addLine();

            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}


