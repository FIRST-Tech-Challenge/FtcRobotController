package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.common.AbstractOpMode;

@TeleOp(name="webcam calibration")
public class WebcamPipelineCalibration extends AbstractOpMode {
    OpenCvWebcam webcam;
    RingSummationPipeline pipeline;

    double zeroRingAvg = 0;
    double oneRingAvg = 0;
    double fourRingAvg = 0;

    @Override
    protected void onInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new RingSummationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN); //specify cam orientation and calibrate the resolution
            }
        });
    }

    @Override
    protected void onStart() {
        telemetry.addData("press a to start the zero ring calibration", " ");
        telemetry.update();
        while(!gamepad1.a && AbstractOpMode.currentOpMode().opModeIsActive());
        try {
            telemetry.clear();
            telemetry.addData("starting the zero ring calibration", "");
            telemetry.update();
            pipeline.resetMatSumCalibration();
            Thread.sleep(5000);
            zeroRingAvg = pipeline.ringsMatSumAvg;
            telemetry.clear();
            telemetry.addData("Zero ring sum", pipeline.ringsMatSumAvg);
            telemetry.addData("press a to start the one ring calibration", " ");
            telemetry.update();
            while(!gamepad1.a && AbstractOpMode.currentOpMode().opModeIsActive());
            telemetry.clear();
            telemetry.addData("starting the one ring calibration", "");
            telemetry.update();
            pipeline.resetMatSumCalibration();
            Thread.sleep(5000);
            oneRingAvg = pipeline.ringsMatSumAvg;
            telemetry.clear();
            telemetry.addData("One ring sum", pipeline.ringsMatSumAvg);
            telemetry.addData("press a to start the four ring calibration", " ");
            telemetry.update();
            while(!gamepad1.a && AbstractOpMode.currentOpMode().opModeIsActive());
            telemetry.clear();
            telemetry.addData("starting the four ring calibration", "");
            telemetry.update();
            pipeline.resetMatSumCalibration();
            Thread.sleep(5000);
            fourRingAvg = pipeline.ringsMatSumAvg;
            telemetry.clear();
            telemetry.addData("Four ring sum", pipeline.ringsMatSumAvg);
            telemetry.addData("press a to display all the values", " ");
            telemetry.update();
            while(!gamepad1.a && AbstractOpMode.currentOpMode().opModeIsActive());
            telemetry.clear();
            telemetry.addData("zero ring: ", zeroRingAvg);
            telemetry.addData("one ring: ", oneRingAvg);
            telemetry.addData("four ring: ", fourRingAvg);
            telemetry.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(opModeIsActive());

    }

    @Override
    protected void onStop() {

    }

    private class RingSummationPipeline extends OpenCvPipeline {
        double ringsMatNetSum;
        double instances;
        double ringsMatSumAvg;
        Mat workingMat;
        @Override
        public Mat processFrame(Mat mat) {
            workingMat = new Mat();
            mat.copyTo(workingMat);
            if(workingMat.empty()){
                return mat;
            }
            Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb); //grayscale the image
            Mat rings = workingMat.submat(190, 240, 20, 80); //cuts a submat from the whole image of what we want
            Imgproc.rectangle(workingMat, new Rect(200, 180, 70, 60), new Scalar(0, 255, 0));
            double ringsMatSum = Core.sumElems(rings).val[2];
            ringsMatNetSum += ringsMatSum;
            instances++;
            ringsMatSumAvg = ringsMatNetSum / instances;
            return workingMat;
        }

        public void resetMatSumCalibration(){
            ringsMatSumAvg = 0;
            ringsMatNetSum = 0;
            instances = 0;
        }
    }

}
