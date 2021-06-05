package org.firstinspires.ftc.robot_utilities;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

public class VisionController {

    OpenCvInternalCamera phoneCam;
    UGContourRingPipeline pipeline;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public VisionController(OpenCvInternalCamera phoneCam) {
        this.phoneCam = phoneCam;
        pipeline = new UGContourRingPipeline();
        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        UGContourRingPipeline.Config.setHORIZON(Vals.vision_horizon);
        this.phoneCam.setPipeline(pipeline);
        this.phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        this.phoneCam.openCameraDeviceAsync(() -> this.phoneCam.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN));
    }

    public String getHeight() {
        return "" + pipeline.getHeight();
    }
    public int getRingPosition() {
        UGContourRingPipeline.Height height = pipeline.getHeight();

        switch (height) {
            case ZERO:
                return 0;
            case ONE:
                return 1;
            case FOUR:
                return 4;
        }

        return -1;
    }

//    private static class SkystoneDeterminationPipeline extends OpenCvPipeline {
//        /*
//         * An enum to define the skystone position
//         */
//
//        public enum RingPosition
//        {
//            FOUR,
//            ONE,
//            NONE
//        }
//
//        /*
//         * Some color constants
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//
//        /*
//         * The core values which define the location and size of the sample regions
//         */
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30,240);
//
//        static final int REGION_WIDTH = 35;
//        static final int REGION_HEIGHT = 25;
//
//        final int FOUR_RING_THRESHOLD = 150;
//        final int ONE_RING_THRESHOLD = 135;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        private volatile RingPosition position = RingPosition.FOUR;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame) {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            position = RingPosition.FOUR; // Record our analysis
//            if(avg1 > FOUR_RING_THRESHOLD){
//                position = RingPosition.FOUR;
//            }else if (avg1 > ONE_RING_THRESHOLD){
//                position = RingPosition.ONE;
//            }else{
//                position = RingPosition.NONE;
//            }
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            return input;
//        }
//
//        public int getAnalysis() {
//            return avg1;
//        }
//    }
}
