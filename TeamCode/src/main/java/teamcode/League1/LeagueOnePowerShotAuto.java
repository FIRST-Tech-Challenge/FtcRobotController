package teamcode.League1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Vector2D;

@Autonomous(name= "Powershot")
public class LeagueOnePowerShotAuto extends AbstractOpMode {
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
    NumRings position;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Point(25, 25), 0); //calibrate this
        movement = new PurePursuitMovement(localizer);
        drivetrain = new MecanumDriveTrain(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        DetectionAlgorithm detector = new DetectionAlgorithm();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
        allPoints = new ArrayList<>(); //IT IS CRITICAL WHEN BUILDING THIS LIST to add the starting position of the robot, this way the framework works as intended
        allPoints.add(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(50, 25, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(50, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(25, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));

    }

    enum NumRings{
        ZERO, ONE, FOUR
    }

    private class DetectionAlgorithm extends OpenCvPipeline{
        Mat workingMat;


        //need to calibrate all these values
        //min sum values of the mats in each of the 3 cases
        private final double RINGS_ZERO_SUM = 0;
        private final double RINGS_ONE_SUM = 1;
        private final double RINGS_FOUR_SUM = 4;


        @Override
        public Mat processFrame(Mat input) {
            workingMat = new Mat();
            input.copyTo(workingMat);
            if(workingMat.empty()){
                return input;
            }
            Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb); //grayscale the image
            Mat rings = workingMat.submat(120, 150, 10, 50); //cuts a submat from the whole image of what we want


            Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0)); //draws a rectangle for debugging
            // so we can see what the robot is looking at (the Submat)

            double ringsMatSum = Core.sumElems(rings).val[2]; //sums the mats contents and from that we can deduce based on the sum how much orange
            //is in the frame, using the conditional logic below we can deduce the stack size and therefore auto zone

            if(ringsMatSum >= RINGS_ZERO_SUM && ringsMatSum < RINGS_ONE_SUM){
                position = NumRings.ZERO;
            }else if(ringsMatSum >= RINGS_ONE_SUM && ringsMatSum < RINGS_FOUR_SUM){
                position = NumRings.ONE;
            }else if(ringsMatSum >= RINGS_FOUR_SUM){
                position = NumRings.FOUR;
            }


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
        movement.initPath(path);
        while(movement.isActive || opModeIsActive()){
            movement.followCurve(path, 0); //path is the path and follow angle is in DEGREES
            drivetrain.setPower(new Vector2D(MovementVars.movementX, -MovementVars.movementY), MovementVars.movementTurn);
        }
    }

    //prints to a file in the format
    // x y rad
    @Override
    protected void onStop() {

        Point currentPos = localizer.getCurrentPosition();
        double currentAng = localizer.getGlobalRads();
        File f = new File(Constants.SAVE_FILE_PATH);
        try {
            PrintStream ps = new PrintStream(f);
            ps.println(currentPos.toStringData() + " " + currentAng);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }



}
