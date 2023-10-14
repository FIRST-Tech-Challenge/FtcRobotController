package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//TODO: copy libOpenCvAndroid453.so from /doc/native_libs to FIRST on robot controller

@TeleOp(name="Image Collection", group="Collection")
public class ImageCollection extends PPRobot {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    boolean ready = false;
    boolean streaming = true;

    boolean saveImage = false;

    enum Labels {
        ONEDOT(0),
        TWODOTS(1),
        THREEDOTS(2);

        private int assignedNum;

        Labels(int aN){
            assignedNum = aN;
        }

        public int getAssignedNum(){
            return assignedNum;
        }
    }

    Labels currentLabel = Labels.ONEDOT;

    @Override
    public void init(){
        super.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error code: " + errorCode);
                telemetry.update();
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void loop(){
        super.loop();
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        if(ready){
            telemetry.addData("Zoom: ", webcam.getPtzControl().getZoom());
            telemetry.addData("Max zoom: ", webcam.getPtzControl().getMaxZoom());
            telemetry.addData("Min zoom: ", webcam.getPtzControl().getMinZoom());
        }
        telemetry.addData("Current Label: ", currentLabel.toString());

        if (getEnhancedGamepad1().isaJustPressed()){
            telemetry.addLine("A Pressed");
            saveImage = true;
            pipeline.saveMatToDisk(pipeline.getMat(), "mat");
//            webcam.stopStreaming();
        }
        else if(getEnhancedGamepad1().isbJustPressed()){
            telemetry.addLine("B Pressed");
            webcam.startStreaming(640, 480);
            streaming = true;
        }
        else if(getEnhancedGamepad1().isxJustPressed()){
            telemetry.addLine("X Pressed");
            webcam.stopStreaming();
            streaming = false;
        }
        else if (getEnhancedGamepad1().isyJustPressed()){
            telemetry.addLine("Y Pressed");
            switch (currentLabel) {
                case ONEDOT:
                    currentLabel = Labels.TWODOTS;
                    break;
                case TWODOTS:
                    currentLabel = Labels.THREEDOTS;
                    break;
                case THREEDOTS:
                    currentLabel = Labels.ONEDOT;
                    break;
            }
        }
        if(streaming) {
            telemetry.addLine("Webcam is Streaming");
        }
        else{
            telemetry.addLine("Webcam is not Streaming");
        }
        telemetry.update();
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * This is the mat object into which we will save the mat passed by the backend to processFrame()
         * It will be returned by getMat()
         */
        Mat mat = new Mat();
        private int[] labelTrackers = {8, 8, 8};

        @Override
        public Mat processFrame(Mat input) {
            Size dim = new Size(320, 320);
            Imgproc.resize(input, mat, dim);

            Size dim2 = new Size(160, 160);
            Imgproc.resize(mat, mat, dim2);
            //New Robot Rect Dimensions: x:80, y:80, width:68, height:68
            //Rect rectCrop = new Rect(65, 40, 35, 68);
            //mat = new Mat(mat, rectCrop);

//            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA);
            if(/* num % 10 == 0 && num <= 200 */ saveImage) {
                String name = String.format("(%s)(%d)", currentLabel.toString(), ++labelTrackers[currentLabel.getAssignedNum()]);
                saveMatToDisk(mat, name);
                saveImage = false;
            }
            return mat;
        }

        @Override
        public void onViewportTapped(){
            viewportPaused = !viewportPaused;

            if (viewportPaused){
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }

        Mat getMat(){
            return mat;
        }
    }
}