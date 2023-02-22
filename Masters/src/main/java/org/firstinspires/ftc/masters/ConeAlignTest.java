package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;

@Autonomous(name="Cone Align Test", group="demo")
public class ConeAlignTest extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    TelemetryPacket packet = new TelemetryPacket();

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    @Override
    public void runOpMode()
    {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamSleeve"), cameraMonitorViewId);
        //OpenCV Pipeline
        CAMShiftPipelinePowerPlay myPipeline;
        webcam.setPipeline(myPipeline = new CAMShiftPipelinePowerPlay(telemetry,packet));
        // Configuration of Pipeline
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();

        packet.put("Position", 3);
        dashboard.sendTelemetryPacket(packet);

        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CAMShiftPipelinePowerPlay.DetectedObject detectedObject = null;
        Size size = null;
        Point center = null;
        int numCones;
        CAMShiftPipelinePowerPlay.ConeOrientation orientation;

        waitForStart();


        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            detectedObject = myPipeline.detectedObject;
            size = myPipeline.size;
            center = myPipeline.center;
            numCones = myPipeline.numCones;
            orientation = myPipeline.orientation;

            telemetry.addData("Position", detectedObject);
            telemetry.update();
        }



        //Center to Pole 14.5 in
        //Webcam to Pole 15 in
        //Pole width 1 in



//        while (opModeIsActive())
//        {
//
////            while (myPipeline.center.x <= 190 && myPipeline.center.x >= 220) {
//                telemetry.addData("Center X:", myPipeline.center.x);
//                if (myPipeline.center.x <= 200) {
//                    leftFrontMotor.setPower(-.14);
//                    rightFrontMotor.setPower(.14);
//                    leftRearMotor.setPower(-.14);
//                    rightRearMotor.setPower(.14);
//                }
//                else if (myPipeline.center.x >= 220) {
//                    leftFrontMotor.setPower(.14);
//                    rightFrontMotor.setPower(-.14);
//                    leftRearMotor.setPower(.14);
//                    rightRearMotor.setPower(-.14);
//                } else {
//                    leftFrontMotor.setPower(0);
//                    rightFrontMotor.setPower(0);
//                    leftRearMotor.setPower(0);
//                    rightRearMotor.setPower(0);
//                }
//                telemetry.update();
////            }
////            telemetry.addData("Width:", myPipeline.size.width);
////            telemetry.update();
////
////            if (myPipeline.size.width <= 63) {
////               leftFrontMotor.setPower(.15);
////               rightFrontMotor.setPower(.15);
////               leftRearMotor.setPower(.15);
////               rightRearMotor.setPower(.15);
////           } else if (myPipeline.size.width >= 69) {
////               leftFrontMotor.setPower(-.15);
////               rightFrontMotor.setPower(-.15);
////               leftRearMotor.setPower(-.15);
////               rightRearMotor.setPower(-.15);
////           } else {
////               leftFrontMotor.setPower(0);
////               rightFrontMotor.setPower(0);
////               leftRearMotor.setPower(0);
////               rightRearMotor.setPower(0);
//           }
//        }
    }
}