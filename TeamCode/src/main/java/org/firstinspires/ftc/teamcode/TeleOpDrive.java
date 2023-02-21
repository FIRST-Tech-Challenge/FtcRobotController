package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PoseHolder.HIGH_TICKS;
import static org.firstinspires.ftc.teamcode.PoseHolder.INTAKE_TICKS;
import static org.firstinspires.ftc.teamcode.PoseHolder.LOW_TICKS;
import static org.firstinspires.ftc.teamcode.PoseHolder.MID_TICKS;
import static org.firstinspires.ftc.teamcode.PoseHolder.START_TICKS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "FinalDrive", group = "Taus")
//@Disabled
public class TeleOpDrive extends LinearOpMode{

    private SampleMecanumDrive drive;
    private ManipulatorMethods manipulator;

    private PoleDetectionPipeline opencv = null;

    private double centerPosX = 0;
    OpenCvWebcam webcam = null;

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POLE
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(0, 0);


    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        manipulator = new ManipulatorMethods(hardwareMap);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        opencv = new PoleDetectionPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(opencv);
                //start streaming the camera
                //
                // webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseHolder.currentDrivePose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        currentMode = Mode.NORMAL_CONTROL;

        waitForStart();

        telemetry.addLine("Running");

        while (opModeIsActive()) {

            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();

            telemetry.addData("mode", currentMode);

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            //DRIVEBASE
            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    /*if (gamepad1.dpad_left) {
                        currentMode = Mode.ALIGN_TO_POLE;
                    }*/

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    );
                    break;
                case ALIGN_TO_POLE:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.dpad_right) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

            /***********************************************************************************************************************************/
            //INTAKE
            if (gamepad1.right_trigger > 0.2) {
                manipulator.intake();
            } else if (gamepad1.left_trigger > 0.2) {
                manipulator.outtake();
            } else {
                manipulator.stopIntake();
            }


            /***********************************************************************************************************************************/
            //DRIVE WITH VISION
            /*while (gamepad1.left_bumper) {
                double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
                double poleX = opencv.poleCenterX;
                double errorX = poleX - 640;
                telemetry.addData("poleX: ", poleX);
                telemetry.addData("angle: ", errorX / PIXELS_PER_DEGREE);
                telemetry.update();


                // if pole is on the right of the screen turn right
                if (errorX < -150) {
                    drive.setPowerOfIndividualMotorsTo(0.4, -0.4, 0.4, -0.4);
                } else if (errorX > 150) {
                    drive.setPowerOfIndividualMotorsTo(-0.4, 0.4, -0.4, 0.4);
                } else {
                    drive.setPowerOfAllMotorsTo(0);
                }
            }*/

//            double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
//            double poleX = opencv.poleCenterX;
//            double errorX = poleX - 640;
//            while (gamepad1.right_bumper) {
//                if ( poleX < 550 || poleX > 750) {
////                  double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
//                    poleX = opencv.poleCenterX;
//                    errorX = poleX - 640;
//                    telemetry.addData("poleXXXXZX" + ": ", poleX);
//                    telemetry.addData("angle: ", errorX / PIXELS_PER_DEGREE);
//                    telemetry.addLine("test");
//                    telemetry.addData("distance: ", manipulator.distanceSensor.getDistance(DistanceUnit.CM));
//                    telemetry.update();
//
//                    // if pole is on the right of the screen turn right
//                    //drive.turnWithPID((int)(errorX / PIXELS_PER_DEGREE), 1, 1, 1, 1);
//
//                    //drive.turnWithPID(drive.getAngle() - (errorX / PIXELS_PER_DEGREE), 1, 0, 0.0001, 1);
//                    drive.rotate((int) (errorX / PIXELS_PER_DEGREE), -0.3);
//                    telemetry.addData("Error: ", errorX);
//                }
//                drive.setPowerOfAllMotorsTo(0);
//                sleep(1000);
//            }
//
//            if (gamepad1.dpad_left) {
//                drive.turnWithPID(90, 1, 0,0.0001, 1);
//            }
//            if (gamepad1.dpad_right) {
//                drive.turnWithPID(-90, 1, 0, 0.0001, 1);
//            }
//            if (gamepad1.dpad_up) {
//                drive.turnWithPID(0, 1, 0, 0.0001, 1);
//            }
//            if (gamepad1.dpad_down) {

//                drive.turnWithPID(180, 1, 0, 0.0001, 1);
//            }
//
//            drive.move(gamepad1);


            /***********************************************************************************************************************************/
            //SLIDES
            if(gamepad2.right_stick_button) {
                manipulator.resetSlides();
            }
            if (gamepad2.dpad_up) {
                manipulator.moveSlides(0.9);
            } else if (gamepad2.dpad_down) {
                manipulator.moveSlides(-0.9);
            } else {
                if (gamepad1.y || (gamepad2.left_trigger > 0.2)) {
                    manipulator.moveSlideEncoder(START_TICKS,0.9
                    );
                } else if (gamepad1.a) {
                    manipulator.moveSlideEncoder(INTAKE_TICKS,0.9);
                } else if (gamepad2.a) {
                    manipulator.moveSlideEncoder(LOW_TICKS,0.9);
                } else if (gamepad2.b) {
                    manipulator.moveSlideEncoder(MID_TICKS,0.9);
                } else if (gamepad2.y) {
                    manipulator.moveSlideEncoder(HIGH_TICKS,0.9);
                } else if (manipulator.rightSlides.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    manipulator.moveSlideEncoder(manipulator.leftSlides.getCurrentPosition(), manipulator.rightSlides.getCurrentPosition(), 0.5);
                }
            }

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);


            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            drive.getLocalizer().update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Right Slides Encoder: ", manipulator.rightSlides.getCurrentPosition());
            telemetry.addData("Left Slides Encoder: ", manipulator.leftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
