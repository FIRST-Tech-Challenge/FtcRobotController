package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.test.AyushVision.NumRings;
import teamcode.test.AyushVision.RingDetectionPipeline;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;



public class LeagueOnePowerShotAuto {
    /*
    this is an example script to showcase various autonomous elements
    such as use of the pure pursuit framework and the localizer as well the vision
     */
//    MecanumDriveTrain drivetrain;
//    Localizer localizer;
//    PurePursuitMovement movement;
//    ArrayList<CurvePoint> allPoints;
//    OpenCvWebcam webcam;
//    Shooter shooter;
//    WobbleArm arm;
//    NumRings rings;
//    Thread driveThread, armThread, driveCommandThread; // more accurate names are dependent on drive and independent on drive, (being drive and arm respectively)
//    boolean isPurePursuit;
//
//    @Override
//    protected void onInitialize() {
//        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0); //calibrate this
//        //movement = new PurePursuitMovement(localizer);
//        drivetrain = new MecanumDriveTrain(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//        arm = new WobbleArm(hardwareMap, true);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//        RingDetectionPipeline pipeline = new RingDetectionPipeline();
//        webcam.setPipeline(pipeline);
//        isPurePursuit = true;
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN); //specify cam orientation and calibrate the resolution
//            }
//        });
//        driveThread = new Thread(){
//            @Override
//            public void run(){
//                driveSequence();
//            }
//        };
//        armThread = new Thread(){
//            public void run(){
//                try {
//                    Thread.sleep(5000);
//
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//
//            }
//        };
//        driveCommandThread = new Thread(){
//            public void run(){
//                while(opModeIsActive()) {
//                    if(isPurePursuit) {
//                        drivetrain.setPowerPurePursuit(new Vector2D(MovementVars.movementX, -MovementVars.movementY), MovementVars.movementTurn);
//                    }
//                }
//            }
//        };
//        while(!opModeIsActive()){
//            rings = pipeline.getRings();
//            telemetry.addData("Rings:" ,rings);
//            RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentPosition();
//            // todo have a toString for state
//            telemetry.addData("Localizer pos", state.getPosition());
//            telemetry.addData("localizer rads", state.getRotation());
//            telemetry.update();
//        }
//        allPoints = new ArrayList<>(); //IT IS CRITICAL WHEN BUILDING THIS LIST to add the starting position of the robot, this way the framework works as intended
//        //allPoints.add(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        //allPoints.add(new CurvePoint(64.5, 0, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        //allPoints.add(new CurvePoint(50, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//        //allPoints.add(new CurvePoint(25, 50, 1.0, 0.5, 10, Math.toRadians(50), 1.0));
//
//    }
//    private class VirtualController {
//        // speed in inches per second
//        private double requestedSpeed = 0;
//        // starting board location in inches
//        private Point startingPosition = new Point(0,0);
//        // starting board rotation, in degrees
//        private double startingRotation = 0;
//        VirtualController() {
//
//        }
//        void setSpeed(double inchesPerSecond) {
//            requestedSpeed = inchesPerSecond;
//        }
//        void shoot(int count, double distanceInches) {
//            // TODO
//        }
//        void moveTo(Point targetPosition, double targetRotation) {
//
//        }
//    }
//    private void driveSequence() {
//
//        //movemet.
//        //shooter.shoot(1, 1430);
////        controller.setSpeed(1);
////        controller.setStartingPosition(new Point(120, 10), 0);
////        controller.shoot(3, 120);
////        controller.moveTo(new Point(120, 24), -90);
//        /*
//        movement.isActive = true;
//        shooter.shoot(1, 1400);
//        while(movement.isActive && opModeIsActive()) {
//            movement.followCurve(new CurvePoint(0, 0,0.5,0,2, 0,0),
//                    new CurvePoint(2, 0,0.5,0,3, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }
//
//        movement.isActive = true;
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,0.5,0,3, 0,0),
//                    new CurvePoint(2, -6.5,0.5,0,3, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }
//        isPurePursuit = false;
//        drivetrain.rotate(0.01, 0.5, true);
//        isPurePursuit = true;
//        shooter.shoot(1, 1400);
//        Utils.sleep(300);
//        movement.isActive = true;
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,0.5,0,1, 0,0),
//                    new CurvePoint(2, -12.5,0.5,0,1, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }
//        isPurePursuit = false;
//        drivetrain.rotate(0.01, 0.5, true);
//        isPurePursuit = true;
//        shooter.shoot(1,1400);
//
//        movement.isActive = true;
//        //55,-20
////        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
////            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0,2, 0,0),
////                    new CurvePoint(55, -18,1,0,2, 0,0), 0);
////            telemetry.addData("current pos", localizer.getCurrentPosition());
////            telemetry.addData("current rot", localizer.getGlobalRads());
////            telemetry.update();
////        }
//        Debug.log("here");
//        isPurePursuit = false;
//        drivetrain.rotate(0.0, 0.5, false);
//        isPurePursuit = true;
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,0.8,0,1, 0,0),
//                    new CurvePoint(92, localizer.getCurrentPosition().y,0.8,0,1, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }
////        //goTo(firstPoint, 1.0,0.5);
////        isPurePursuit = false;
////        //shooter.setFlywheelVelocity(1300, AngleUnit.DEGREES);
////        drivetrain.rotate(-0.03,-0.5, false);
////        isPurePursuit = true;
////        //shooter.shoot(1, 1300);
//////        shooter.progressBanana();
//////        Utils.sleep(250);
//////        shooter.progressBanana();
////        movement.isActive = true;
//////        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//////            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,0.5,0,2, 0,0),
//////                    new CurvePoint(55, -13,0.5,0,2,  0,0), 0);
//////            telemetry.addData("current pos", localizer.getCurrentPosition());
//////            telemetry.addData("current rot", localizer.getGlobalRads());
//////            telemetry.update();
//////        }
//////        //rotate(-PI/30.0,-0.5);
//////        isPurePursuit = false;
//////        drivetrain.rotate(-0.15,-0.5);
//////        isPurePursuit = true;
//////        shooter.shoot(1, 1300);
//////        movement.isActive = true;
//////        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//////            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,0.5,0,2, 0,0),
//////                    new CurvePoint(55, -8,0.5,0,2, 0,0), 0);
//////            telemetry.addData("current pos", localizer.getCurrentPosition());
//////            telemetry.addData("current rot", localizer.getGlobalRads());
//////            telemetry.update();
//////        }
////        isPurePursuit = false;
////        drivetrain.rotate(-0.12,-0.5, false);
////        Utils.sleep(1000);
////        shooter.shoot(1, 1300);
//////        Utils.sleep(250);
//////        shooter.progressBanana();
////        drivetrain.rotate(-0.2, -0.5, false);
////        Utils.sleep(1000);
////        shooter.shoot(1, 1300);
////        isPurePursuit = true;
////
////        drivetrain.rotate(0, 0.5, false);
////
//////        //4 rings
//////        movement.isActive = true;
//////        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//////            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0,2, 0,0),
//////                    new CurvePoint(55, 0,0.7,0,2, 0,0), 0);
//////            telemetry.addData("current pos", localizer.getCurrentPosition());
//////            telemetry.addData("current rot", localizer.getGlobalRads());
//////            telemetry.update();
//////        }
//////        isPurePursuit = false;
//////        drivetrain.rotate(-0.1, 0.5);
//////        isPurePursuit = true;
//////        shooter.shoot(1, 1300);
//        movement.isActive = true;
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0,2, 0,0),
//                    new CurvePoint(localizer.getCurrentPosition().x, 42,0.7,0,2, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }
//
//        if(rings == NumRings.ZERO){
//            movement.isActive = true;
//            while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//                movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0,2, 0,0),
//                        new CurvePoint(80, localizer.getCurrentPosition().y,0.7,0,2, 0,0), 0);
//                telemetry.addData("current pos", localizer.getCurrentPosition());
//                telemetry.addData("current rot", localizer.getGlobalRads());
//                telemetry.update();
//            }
//
//        }
//
//        movement.isActive = true;
////        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
////            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0,2, 0,0),
////                    new CurvePoint(105, localizer.getCurrentPosition().y,0.7,0,2, 0,0), 0);
////            telemetry.addData("current pos", localizer.getCurrentPosition());
////            telemetry.addData("current rot", localizer.getGlobalRads());
////            telemetry.update();
////        }
//        isPurePursuit=false;
////        //Debug.log("rings: " + rings);
//        if(rings == NumRings.FOUR){
//            drivetrain.rotate(Math.PI, -0.5, true);
//        }else if(rings == NumRings.ONE){
//            drivetrain.rotate(-Math.PI/2.0, -0.5, true);
//            //rotate
//        }
//        isPurePursuit = true;
//        arm.runToPosition(-4000, 0.5);
//        arm.adjustClaw();
//        movement.isActive = true;
//        double direction = 1;
//        if(rings==NumRings.FOUR) {
//            Utils.sleep(500);
//            arm.runToRelativePosition(0, 0.5);
//            direction = 1;
//            movement.isActive = true;
//            while (movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//                movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1, 0, 1, 0, 0),
//                        new CurvePoint(72, localizer.getCurrentPosition().y, 0.7, 0, 1, 0, 0), 0);
//                telemetry.addData("current pos", localizer.getCurrentPosition());
//                telemetry.addData("current rot", localizer.getGlobalRads());
//                telemetry.update();
//            }
//        }else if(rings== NumRings.ONE){
//            Utils.sleep(500);
//            arm.runToRelativePosition(0, 0.5);
//            movement.isActive = true;
//            isPurePursuit = false;
//            drivetrain.rotate(0, 0.5, true);
//            isPurePursuit = true;
//            while (movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//                movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1, 0, 1, 0, 0),
//                        new CurvePoint(72, localizer.getCurrentPosition().y, 0.7, 0, 1, 0, 0), 0);
//                telemetry.addData("current pos", localizer.getCurrentPosition());
//                telemetry.addData("current rot", localizer.getGlobalRads());
//                telemetry.update();
//            }
////            while (movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
////                movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y, 1, 0, 1, 0, 0),
////                        new CurvePoint(125, localizer.getCurrentPosition().y, direction *0.7, 0, 1, 0, 0), 0);
////                telemetry.addData("current pos", localizer.getCurrentPosition());
////                telemetry.addData("current rot", localizer.getGlobalRads());
////                telemetry.update();
////            }
//            direction = 1;
////
//       }
//
//        //drivetrain.rotate(-0.05, -0.5);
//*/
//    }
//
//
//
//    /*
//    this method should simply drive the robot in the pattern shown below
//
//
//    <-<-<-<-
//            ^
//            |
//            ^
//            |
//    ->->->->
//    the most important thing to note here is YOU MUST ADD
//     THE START POSITION OF THE ROBOT TO THE LIST THE METHOD WILL CRASH IF YOU DONT
//     */
//
//    @Override
//    protected void onStart() {
//        //Drive Thread
//        /*
//        This thread is telling the robot to repeatedly recalculate its power based on some math
//        and reset the power as a sligthly different vector each time resulting in humanoid movements
//        this process is called Pure Pursuit FTC team 11115 did an excellent video series on the subject
//        if you are interested
//         */
//        try {
//            webcam.stopStreaming(); // may not be necessary if we are processing realtime location of the balls on the field
//            //Debug.log(rings);
//            //arm.runToPosition(WobbleConstants.RETRACTED_POSITION, 0.5);
//            localizer.start();
//            driveThread.start();
//            driveCommandThread.start();
//            //armThread.start();
//            //drivetrain.rotate(Math.PI,0.5);
//            //Point secondtPoint = new Point(128,0);
//
//
//            //followPath(allPoints);
//            //this is my fix to Spline entanglement. Essentially if that comes up parse the path into
//            // another list and simply call a line like below, from there the method should cover the rest
//            //This may make some minor oscillations however with smart pathing we can avoid this and make a fluid auto path
//            //that will not have very much pauses
//            //followPath(secondPath);
//            while (opModeIsActive()) ;
//        } finally {
//            localizer.stopThread();
//        }
//    }
//
//
//    //Arm Thread
//        /*
//        This thread should have a series of wait (Utils.sleep(long millis)) commands in followed by some
//        scoring commands or manipulative commands that allow for the robot to progress the state of
//        scoring
//         */
//
//        //this is here to stall the main thread so the OpMode does not prematurely end
//
//    private void followPath(ArrayList<CurvePoint> path) {
//        movement.initPath(path);
//        while(movement.isActive && opModeIsActive()){
//            movement.followCurve(path, 0); //path is the path and follow angle is in DEGREES
//            drivetrain.setPower(new Vector2D(MovementVars.movementX, -MovementVars.movementY), MovementVars.movementTurn);
//        }
//    }
//
//    //prints to a file in the format
//    // x y rad
//    @Override
//    protected void onStop() {
//
////        Point currentPos = localizer.getCurrentPosition();
////        double currentAng = localizer.getGlobalRads();
////        File f = new File(Constants.SAVE_FILE_PATH);
////        try {
////            PrintStream ps = new PrintStream(f);
////            ps.println(currentPos.toStringData() + " " + currentAng);
////        } catch (FileNotFoundException e) {
////            e.printStackTrace();
////        }
//    }
//
//

}
