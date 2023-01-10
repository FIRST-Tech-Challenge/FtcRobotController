package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

@Autonomous(name="TestingAuto", group="Test")
public class TestingAuto extends automethods {
    HardwarePushbot robot = new HardwarePushbot();// Use a Pushbot's hardware


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.autoinit(hardwareMap);
        /* webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new BarcodePositionDetector.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
       /* int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
*/


        waitForStart();

////////////////////////////////////ROBOT  START///////////////////////////////////////////////////

        telemetry.update();
        encoderDrive(.9,60, 10);
        imuTurn(.35,-270);
        imuHold(3);
      //  setLevel(3);
       // encoderDrive(.3,3,5);
      //  openClaw();
       // encoderDrive(.3,-4,5);
       // setLevel(0);
        //imuTurn(.7,-180);
        strafeRight(.7,-10,5);
        /*encoderDrive(.7,10,5);
        closeclaw();
        encoderDrive(.7,-10,5);
        strafeRight(.7,10,5);
        imuTurn(.7,180);
        setLevel(3);
        encoderDrive(.3,4,3);
        openClaw();
        encoderDrive(.7,-4, 3);
        strafeRight(.7,20,5);
        */
        //webcam


        //  rpos = returnRingPosition(3);





    }}