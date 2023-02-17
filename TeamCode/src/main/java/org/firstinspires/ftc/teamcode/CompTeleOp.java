package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "FinalDrive", group = "Taus")
//@Disabled
public class CompTeleOp extends LinearOpMode{

    int START_TICKS = (int)(271 / 1.5);
    int INTAKE_TICKS = 0;
    int LOW_TICKS = (int)(2063 / 1.5);
    int MID_TICKS = (int)(3500 / 1.5);
    int HIGH_TICKS = (int)(4900 / 1.5);

    private Hardware robot = new Hardware(false);
    private DrivebaseMethods drive = new DrivebaseMethods(robot);
    private ManipulatorMethods manipulator = new ManipulatorMethods(robot);

    private PoleDetectionPipeline opencv = null;
    private double centerPosX = 0;
    OpenCvWebcam webcam = null;

    public void runOpMode() {
        robot.initializeHardware(hardwareMap);



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
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });


        drive.resetAngle();

        waitForStart();

        telemetry.addLine("Running");

        while (opModeIsActive()) {


            //DRIVEBASE
            /*if (gamepad1.right_trigger > 0.5) {
                drive.move(gamepad1, 0.3);
            } else {
                drive.move(gamepad1);
            }*/

            //drive.fieldCentric(gamepad1);

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
            while (gamepad1.left_bumper) {
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
            }

            double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
            double poleX = opencv.poleCenterX;
            double errorX = poleX - 640;
            while (gamepad1.right_bumper) {
                if ( poleX < 550 || poleX > 750) {
//                  double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
                    poleX = opencv.poleCenterX;
                    errorX = poleX - 640;
                    telemetry.addData("poleXXXXZX" + ": ", poleX);
                    telemetry.addData("angle: ", errorX / PIXELS_PER_DEGREE);
                    telemetry.addLine("test");
                    telemetry.addData("distance: ", robot.distanceSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();

                    // if pole is on the right of the screen turn right
                    //drive.turnWithPID((int)(errorX / PIXELS_PER_DEGREE), 1, 1, 1, 1);

                    //drive.turnWithPID(drive.getAngle() - (errorX / PIXELS_PER_DEGREE), 1, 0, 0.0001, 1);
                    drive.rotate((int) (errorX / PIXELS_PER_DEGREE), -0.3);
                    telemetry.addData("Error: ", errorX);
                }
                drive.setPowerOfAllMotorsTo(0);
                sleep(1000);
            }

            if (gamepad1.dpad_left) {
                drive.turnWithPID(90, 1, 0,0.0001, 1);
            }
            if (gamepad1.dpad_right) {
                drive.turnWithPID(-90, 1, 0, 0.0001, 1);
            }
            if (gamepad1.dpad_up) {
                drive.turnWithPID(0, 1, 0, 0.0001, 1);
            }
            if (gamepad1.dpad_down) {
                drive.turnWithPID(180, 1, 0, 0.0001, 1);
            }

            drive.move(gamepad1);


            /***********************************************************************************************************************************/
            //SLIDES
            if(gamepad2.x) {
                manipulator.resetSlides();
            }
            if (gamepad2.dpad_up) {
                manipulator.moveSlides(0.9);
            } else if (gamepad2.dpad_down) {
                manipulator.moveSlides(-0.9);
            } else {
                if (gamepad1.y) {
                    manipulator.moveSlideEncoder(START_TICKS,0.9);
                } else if (gamepad1.a) {
                    manipulator.moveSlideEncoder(INTAKE_TICKS,0.9);
                } else if (gamepad2.a) {
                    manipulator.moveSlideEncoder(LOW_TICKS,0.9);
                } else if (gamepad2.b) {
                    manipulator.moveSlideEncoder(MID_TICKS,0.9);
                } else if (gamepad2.y) {
                    manipulator.moveSlideEncoder(HIGH_TICKS,0.9);
                } else if (robot.rightSlides.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    manipulator.moveSlideEncoder(robot.leftSlides.getCurrentPosition(), robot.rightSlides.getCurrentPosition(), 0.5);
                }
            }


            telemetry.addData("Right Slides Encoder: ", robot.rightSlides.getCurrentPosition());
            telemetry.addData("Left Slides Encoder: ", robot.leftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
