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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

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
        ALIGN_RIGHT,
        ALIGN_LEFT,
        ALIGN_UP,
        ALIGN_DOWN,
        ALIGN_TO_POLE
    }


    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);


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
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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
        ElapsedTime headingTurnTimer = new ElapsedTime();

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

            // Declare a target vector you'd like your bot to align with
            // Can be any x/y coordinate of your choosing
            Vector2d RightDpadTargetPosition = new Vector2d(72, drive.getLocalizer().getPoseEstimate().getY());
            Vector2d LeftDpadTargetPosition = new Vector2d(-72, drive.getLocalizer().getPoseEstimate().getY());
            Vector2d UpDpadTargetPosition = new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), 72);
            Vector2d DownDpadTargetPosition = new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), -72);

            //DRIVEBASE
            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (gamepad1.dpad_left) {
                        headingTurnTimer.reset();
                        currentMode = Mode.ALIGN_LEFT;
                    }
                    if (gamepad1.dpad_right) {
                        headingTurnTimer.reset();
                        currentMode = Mode.ALIGN_RIGHT;
                    }
                    if (gamepad1.dpad_up) {
                        headingTurnTimer.reset();
                        currentMode = Mode.ALIGN_UP;
                    }
                    if (gamepad1.dpad_down) {
                        headingTurnTimer.reset();
                        currentMode = Mode.ALIGN_DOWN;
                    }
                    if (gamepad1.right_bumper) {
                        currentMode = Mode.ALIGN_TO_POLE;
                    }

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y * 0.6,
                            -gamepad1.left_stick_x * 0.6,
                            gamepad1.right_stick_x * 0.6
                    );

                    /*Pose2d poseEstimateController = drive.getPoseEstimate();

                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimateController.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );*/
                    break;
                case ALIGN_RIGHT:
                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = RightDpadTargetPosition.minus(poseEstimate.vec());
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
                    fieldOverlay.strokeCircle(RightDpadTargetPosition.getX(), RightDpadTargetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(RightDpadTargetPosition.getX(), RightDpadTargetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(RightDpadTargetPosition.getX(), RightDpadTargetPosition.getY(), RightDpadTargetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(RightDpadTargetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                    if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {

                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    break;
                case ALIGN_LEFT:
                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput2 = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput2 = fieldFrameInput2.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference2 = LeftDpadTargetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta2 = difference2.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF2 = -fieldFrameInput2.rotated(-Math.PI / 2).dot(difference2) / (difference2.norm() * difference2.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta2);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput2 = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF2)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput2,
                            headingInput2
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(LeftDpadTargetPosition.getX(), LeftDpadTargetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(LeftDpadTargetPosition.getX(), LeftDpadTargetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(LeftDpadTargetPosition.getX(), LeftDpadTargetPosition.getY(), LeftDpadTargetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(LeftDpadTargetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                    if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }


                    break;
                case ALIGN_UP:
                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput3 = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput3 = fieldFrameInput3.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference3 = UpDpadTargetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta3 = difference3.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF3 = -fieldFrameInput3.rotated(-Math.PI / 2).dot(difference3) / (difference3.norm() * difference3.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta3);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput3 = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF3)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput3,
                            headingInput3
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(UpDpadTargetPosition.getX(), UpDpadTargetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(UpDpadTargetPosition.getX(), UpDpadTargetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(UpDpadTargetPosition.getX(), UpDpadTargetPosition.getY(), UpDpadTargetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(UpDpadTargetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                    if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }


                    break;
                case ALIGN_DOWN:
                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput4 = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput4 = fieldFrameInput4.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference4 = DownDpadTargetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta4 = difference4.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF4 = -fieldFrameInput4.rotated(-Math.PI / 2).dot(difference4) / (difference4.norm() * difference4.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta4);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput4 = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF4)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput4,
                            headingInput4
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(DownDpadTargetPosition.getX(), DownDpadTargetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(DownDpadTargetPosition.getX(), DownDpadTargetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(DownDpadTargetPosition.getX(), DownDpadTargetPosition.getY(), DownDpadTargetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(DownDpadTargetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                    if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }


                    break;
                case ALIGN_TO_POLE:
//                    double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
                    double PIXELS_PER_DEGREE = 25;
                    double poleX = opencv.poleCenterX;
                    double poleWidth = opencv.poleWidth;
                    double errorX = poleX - 640;
                    double POLE_DISTANCE_CONSTANT = 0.02;

                    if (centerPosX > 750 || centerPosX < 550) {
                        centerPosX = opencv.poleCenterX;
                        telemetry.addData("PoleCenterX", centerPosX);
                        telemetry.addLine("Not Equal");

                        drive.turn(Math.toRadians((640 - centerPosX) / PIXELS_PER_DEGREE));

                        telemetry.update();
                    } else {
                        centerPosX = opencv.poleCenterX;
                        telemetry.addData("PoleCenterX", centerPosX);
                        telemetry.addLine("EQUAL");

                        telemetry.update();

                        drive.leftFront.setPower(0);
                        drive.rightFront.setPower(0);
                        drive.leftRear.setPower(0);
                        drive.rightRear.setPower(0);
                    }

                    if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    break;

            }

            /***********************************************************************************************************************************/
            //INTAKE
            if (gamepad1.right_trigger > 0.2) {
                manipulator.intake();
                gamepad1.rumble(0,1,100);
            } else if (gamepad1.left_trigger > 0.2) {
                manipulator.outtake();
                gamepad1.rumble(1,0,100);
            } else {
                manipulator.stopIntake();
                gamepad1.stopRumble();
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
                gamepad2.rumble(500);
            }
//            if(gamepad2.right_trigger>0.1)
//                gamepad2.rumble(10000);
            if (gamepad2.dpad_up) {
                manipulator.moveSlides(0.9);
            } else if (gamepad2.dpad_down) {
                manipulator.moveSlides(-0.9);
            } else if (gamepad2.right_bumper){
                manipulator.moveSlideEncoder(manipulator.rightSlides.getCurrentPosition() - 50, 1);

            } else {
                if (gamepad1.y || (gamepad2.left_trigger > 0.2)) {
                    manipulator.moveSlideEncoder(START_TICKS,0.9
                    );
                    gamepad2.rumble(500);
                } else if (gamepad1.a) {
                    manipulator.moveSlideEncoder(INTAKE_TICKS,0.9);
                } else if (gamepad2.a) {
                    manipulator.moveSlideEncoder(LOW_TICKS,0.9);
                    gamepad2.rumble(0.2, 0, 500);
                } else if (gamepad2.b) {
                    manipulator.moveSlideEncoder(MID_TICKS,0.9);
                    gamepad2.rumble(0.5, 0.5, 500);
                } else if (gamepad2.y) {
                    manipulator.moveSlideEncoder(HIGH_TICKS,0.9);
                    gamepad2.rumble(500);
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
