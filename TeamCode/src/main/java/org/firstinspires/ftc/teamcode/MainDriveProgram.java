package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.yise.AprilTagDetector;
import org.firstinspires.ftc.teamcode.yise.DriveColorExample;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Competition drive", group="Linear Opmode")
public class MainDriveProgram extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;
    boolean reverseIntake = false;

    boolean inEndGame = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);

        //LedLights leds = new LedLights(hardwareMap);

        DriveColorExample colorSensors = new DriveColorExample(hardwareMap);

        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //leds.setLed(LedLights.ledStates.BLUE);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            if (!inEndGame && getRuntime() > 90) {
                inEndGame = true;
                //leds.setLed(LedLights.ledStates.ENDGAME);
            }

            /**
             * Driving
             */
            //Check for auto navigation requested by driver, otherwise give manual control to driver
            if (gamepad1.dpad_up) {
                if (rrDrive.getPosition().getX() < -30) {
                    rrDrive.navigateToCorner();
                } else if (rrDrive.getPosition().getX() > 0) {
                    arm.extend(LiftArm.Distance.AUTO);
                    rrDrive.dropPixelNear();
                }
            } else if (gamepad1.dpad_down) {
                if (rrDrive.getPosition().getX() > 0) {
                    arm.extend(LiftArm.Distance.FULL);
                    rrDrive.dropPixelFar();
                }
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                if (rrDrive.getPosition().getX() > 0) {
                    arm.extend(LiftArm.Distance.HALF);
                    rrDrive.dropPixelMid();
                }
            } else {
                rrDrive.updateMotorsFromStick(gamepad1);
                rrDrive.update();
            }

            //If button is pressed on gamepad, calibrate position based on middle april tag
            if (aprilTagDetector.getAprilTag() != null && gamepad2.back) {
                AprilTagDetection detection = aprilTagDetector.getAprilTag();
                rrDrive.calibratePos(detection);

                telemetry.addData("X pos: ", 36.25 + detection.ftcPose.x);
                telemetry.addData("Y pos: ", 55.5 - detection.ftcPose.y);
            }



            /**
             * Intake
             */
            //Check if both slots are full in bucket
            if ((colorSensors.getBackPixelColor() != DriveColorExample.Colors.NONE) && (colorSensors.getFrontPixelColor() != DriveColorExample.Colors.NONE)) {
                reverseIntake = true;
            } else {
                reverseIntake = false;
            }

            if ((gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5)) {
                //If both slot are full, outtake excess pixels
                if (reverseIntake) {
                    intakeSystem.runIntakeSystem(-1);
                } else {
                    intakeSystem.runIntakeSystem(1);
                }
            } else if ((gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5)) {
                //Manual outtake
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                //If no inputs, stop intake
                intakeSystem.runIntakeSystem(0);
            }



            /**
             * Arm slides
             */
            //If driver input extend the slides to different legnths
            if (gamepad2.dpad_up){
                arm.extend(LiftArm.Distance.FULL);
                arm.holdArm();
            } else if (gamepad2.dpad_right){
                arm.extend(LiftArm.Distance.HALF);
                arm.holdArm();
            } else if (gamepad2.dpad_left) {
                arm.extend(LiftArm.Distance.AUTO);
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
             * Slow mode toggle
             */
            //If input released, slow mode can be toggled again. This prevents an infinite loop of toggling.
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
            /*telemetry.addData("Slide: ", arm.getSlidePosition());
            telemetry.addData("Arm pos: ", arm.getHandPosition());
            telemetry.addData("Hand power: ", arm.hand.getPower());*/

            /*telemetry.addData("X: ", rrDrive.getPosition().getX());
            telemetry.addData("Y: ", rrDrive.getPosition().getY());
            telemetry.addData("Heading: ", rrDrive.getPosition().getHeading());*/

            telemetry.addLine();

            /*telemetry.addData("Red color front: ", colorSensors.getRedColor()[0]);
            telemetry.addData("Blue color front: ", colorSensors.getBlueColor()[0]);
            telemetry.addData("Green color front: ", colorSensors.getGreenColor()[0]);
            telemetry.addData("Red color back: ", colorSensors.getRedColor()[1]);
            telemetry.addData("Blue color back: ", colorSensors.getBlueColor()[1]);
            telemetry.addData("Green color back: ", colorSensors.getGreenColor()[1]);*/

            /*telemetry.addData("Back pixel color: ", colorSensors.getBackPixelColor());
            telemetry.addData("Front pixel color: ", colorSensors.getFrontPixelColor());*/
            /*telemetry.addData("Ratio back: ", (colorSensors.getRedColor()[1] + colorSensors.getGreenColor()[1])/colorSensors.getBlueColor()[1]);
            telemetry.addData("Ratio front: ", (colorSensors.getRedColor()[0] + colorSensors.getGreenColor()[0])/colorSensors.getBlueColor()[0]);*/

            telemetry.update();
        }
    }
}


