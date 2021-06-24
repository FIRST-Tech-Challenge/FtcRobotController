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
    PurePursuitMovement movement;
    ArrayList<CurvePoint> allPoints;
    OpenCvWebcam webcam;
    DetectedPosition position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(25, 25), 0); //calibrate this
        //movement = new PurePursuitMovement(localizer);
        drivetrain = new MecanumDriveTrain(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        DetectionAlgorithm detector = new DetectionAlgorithm();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
        allPoints = new ArrayList<>(); //IT IS CRITICAL WHEN BUILDING THIS LIST to add the starting position of the robot, this way the framework works as intended
//        allPoints.add(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        allPoints.add(new CurvePoint(50, 25, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        allPoints.add(new CurvePoint(50, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        allPoints.add(new CurvePoint(25, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));

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

    /*
    this method should simply drive the robot in the pattern shown below


    <-<-<-<-
            ^
            |
            ^
            |
    ->->->->
    the most important thing to note here is YOU MUST ADD
     THE START POSITION OF THE ROBOT TO THE LIST THE METHOD WILL CRASH IF YOU DONT
     */

    @Override
    protected void onStart() {
        //Drive Thread
        /*
        This thread is telling the robot to repeatedly recalculate its power based on some math
        and reset the power as a sligthly different vector each time resulting in humanoid movements
        this process is called Pure Pursuit FTC team 11115 did an excellent video series on the subject
        if you are interested
         */
        webcam.stopStreaming(); // may not be necessary if we are processing realtime location of the balls on the field
        new Thread(){
            public void run(){
                followPath(allPoints);
                //this is my fix to Spline entanglement. Essentially if that comes up parse the path into
                // another list and simply call a line like below, from there the method should cover the rest
                //This may make some minor oscillations however with smart pathing we can avoid this and make a fluid auto path
                //that will not have very much pauses
                //followPath(secondPath);
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

    private void followPath(ArrayList<CurvePoint> path) {
//        movement.initPath(path);
//        while(movement.isActive || opModeIsActive()){
//            movement.followCurve(path, 0); //path is the path and follow angle is in DEGREES
//            drivetrain.setPower(new Vector2D(MovementVars.movementX, -MovementVars.movementY), MovementVars.movementTurn);
//        }
    }

    //prints to a file in the format
    // x y rad
    @Override
    protected void onStop() {

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
