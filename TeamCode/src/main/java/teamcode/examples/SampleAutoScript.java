package teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;

import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Vector2D;
import teamcode.test.CVNew.OpenCVTest;

@Disabled
@Autonomous(name= "example Autonomous")
public class SampleAutoScript extends AbstractOpMode {
    /*
    this is an example script to showcase various autonomous elements
    such as use of the pure pursuit framework and the localizer as well the vision
     */
    //TODO add vision stuff in here too
    MecanumDriveTrain drivetrain;
    Localizer localizer;

    OpenCvWebcam webcam;
    DetectedPosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0, 10); //calibrate this
        drivetrain = new MecanumDriveTrain(hardwareMap, localizer);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        DetectionAlgorithm detector = new DetectionAlgorithm();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
    }

    enum DetectedPosition{
        FIRST_POS, SECOND_POS, THIRD_POS
    }

    private class DetectionAlgorithm extends OpenCvPipeline{
        Mat workingMat;

        @Override
        public Mat processFrame(Mat input) {
            workingMat = new Mat();
            input.copyTo(workingMat);
            if(workingMat.empty()){
                return input;
            }
            Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb); //grayscale the image
            Mat left = workingMat.submat(120, 150, 10, 50);
            Mat center = workingMat.submat(120, 150, 80, 120);
            Mat right = workingMat.submat(120, 150, 150, 190);


            Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMat, new Rect(80, 120, 40, 30), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMat, new Rect(150, 120, 40, 30), new Scalar(0, 255, 0));
            double leftSum = Core.sumElems(left).val[2];
            double centerSum = Core.sumElems(center).val[2];
            double rightSum = Core.sumElems(right).val[2];

            if(leftSum > rightSum && leftSum > centerSum){
                position = DetectedPosition.FIRST_POS;
            }
            if(centerSum > rightSum && centerSum > leftSum){
                position = DetectedPosition.SECOND_POS;
            }
            if(rightSum > centerSum && rightSum > leftSum){
                position = DetectedPosition.THIRD_POS;
            }
            /*
            from here all we do is calibrate the exact values of the submatrices relative to our
            scoring objects and test the discrepency in the summation between the color values of
            the 3 and as add some simple conditional logic to account for it
            Something important to note is that the object next year is most likely a ball so the
            rectangular shape will not be perfect
             */

            return workingMat;
        }
    }



    @Override
    protected void onStart() {
        //Drive Thread

        webcam.stopStreaming(); //may not be necessary if we are processing realtime location of the elements on the field
        //rather you would want to switch to a dynamic processing pipeline in that case
        new Thread(){
            public void run(){
                localizer.start();
                //this command moves 2 feet forward at a rate of 12 inches per second, this is optimized using
                //a proportional integral derivative (PID) control loop which isnt necessary but will greatly minimize slippage
                drivetrain.moveToPosition(new Vector2D(0,24), 12, 0, 0.5);
                //you will essentially keep running things like this until you no longer have points to run to
            }
        }.start();
        //Arm Thread
        /*
        This thread should have a series of wait (Utils.sleep(long millis)) commands in followed by some
        scoring commands or manipulative commands that allow for the robot to progress the state of
        scoring
         */
        new Thread(){
            public void run(){

            }
        }.start();
        while(opModeIsActive()); //this is here to stall the main thread so the OpMode does not prematurely end
    }


    //prints to a file in the format
    // x y rad
    // VERY UNLIKELY that we use this, this is more often used for cleanup steps
    @Override
    protected void onStop() {
        localizer.stopThread();
//        Point currentPos = localizer.getCurrentPosition();
//        double currentAng = localizer.getGlobalRads();
//        File f = new File(Constants.SAVE_FILE_PATH);
//        try {
//            PrintStream ps = new PrintStream(f);
//            ps.println(currentPos.toStringData() + " " + currentAng);
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }
    }



}
