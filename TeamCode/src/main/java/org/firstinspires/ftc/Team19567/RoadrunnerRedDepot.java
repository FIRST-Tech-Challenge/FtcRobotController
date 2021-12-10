package org.firstinspires.ftc.Team19567;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.Team19567.tsePipeline.LOCATION;

@Autonomous(name="Roadrunner Test", group="Linear Opmode")

public class RoadrunnerRedDepot extends LinearOpMode {

    private tsePipeline pipeline = new tsePipeline(telemetry); //Team shipping element OpenCV Pipeline
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private DcMotor linearSlideDC = null;
    private DcMotor carouselDC = null;
    private boolean slowMode = false;
    private Servo releaseServo = null;
    private BNO055IMU imu = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDCFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        linearSlideDC = hardwareMap.get(DcMotor.class, "linearSlideDC");
        carouselDC = hardwareMap.get(DcMotor.class, "carouselDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");

        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV", "OpenCV actually connected wow");
                telemetry.update();

                waitForStart();
                switch (pipeline.getLocation()) {
                    case ALLIANCE_FIRST: {
                        location = LOCATION.ALLIANCE_FIRST;
                        telemetry.addData("OpenCV", "First Level Detected");
                        telemetry.update();
                    }
                    case ALLIANCE_SECOND: {
                        location = LOCATION.ALLIANCE_SECOND;
                        telemetry.addData("OpenCV", "Second Level Detected");
                        telemetry.update();
                    }
                    case ALLIANCE_THIRD: {
                        location = LOCATION.ALLIANCE_THIRD;
                        telemetry.addData("OpenCV", "Third Level Detected");
                        telemetry.update();
                    }
                    default: {
                        location = LOCATION.ALLIANCE_THIRD;
                    }
                }
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV", "OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });


    }
}