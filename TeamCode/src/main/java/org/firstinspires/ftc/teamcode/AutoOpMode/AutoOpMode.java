package org.firstinspires.ftc.teamcode.AutoOpMode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Write a detailed description of the autoClass
 */
//TODO: Copy and Rename Autonomous Mode
@Autonomous(name = "Autonomous World 1", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class AutoOpMode extends LinearOpMode{

        public boolean DEBUG_FLAG = true;

        public DriveTrain driveTrain;
        public Vision vision;

        //public Pose2d startPose = vision.BLUE_ALLIANCE_BLUE_TERMINAL;

        //need to fix vision parking positions
        public Vision.PARKING_LOCATION parkingLocation = Vision.PARKING_LOCATION.PARKPOS1;
        public int loopPickConetoDrop = 0;
        public boolean pickCone = false;

        boolean parked = false ;
        public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

        public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
        public Vision.VISION_IDENTIFIED_TARGET targetZone = Vision.VISION_IDENTIFIED_TARGET.RED;//Set a default vision value
        int targetZoneLevel = 0;

        double af = Vision.ALLIANCE_FACTOR;

        @Override
        public void runOpMode() throws InterruptedException {
            /*Create your Subsystem Objects*/
            driveTrain = new DriveTrain(hardwareMap);


            /* Create Controllers */

            //Key Pay inputs to select Game Plan;
            selectGamePlan();
            vision = new Vision(hardwareMap, activeWebcam);
            af = Vision.ALLIANCE_FACTOR;

            // Initiate Camera on Init.
            vision.activateVuforiaTensorFlow();

            //buildAuto();
            /*
            if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
                buildAutoWarehouse();
            } else { //GameField.startPosition == GameField.START_POSITION.STORAGE
                buildAutoStorage();
            }
             */
            //driveTrain.getLocalizer().setPoseEstimate(startPose);
            //buildAuto();

           //autonomousController.runAutoControl();

            telemetry.addData("Waiting for start to be pressed.", "Robot is ready!");
            telemetry.update();

            if (isStopRequested()) return;

            while (!isStopRequested()) {

                //Run Vuforia Tensor Flow
                targetZone = vision.runVuforiaTensorFlow();
                targetZoneLevel = targetZone.ordinal();

                if (!parked) {
                    //autonomousController.runAutoControl();
                }

                if (DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

                //Game Play is pressed
                while (opModeIsActive() && !isStopRequested() && !parked) {
                    gameTimer.reset();

                    vision.deactivateVuforiaTensorFlow();



                    //buildAuto();


                    //Move to Launching Position
                    parked = true;

                    //Write last position to static class to be used as initial position in TeleOp

                    vision.currentPose = driveTrain.getPoseEstimate();
                    vision.poseSetInAutonomous = true;


                    if (DEBUG_FLAG) {
                        printDebugMessages();
                        telemetry.update();
                    }
                }

            }
            safeWait(100);
            vision.currentPose = driveTrain.getPoseEstimate();
            vision.poseSetInAutonomous = true;
        }

        //Initialize any other TrajectorySequences as desired
        TrajectorySequence trajInitToPickAndDropConeToPark;

        //Initialize any other Pose2d's as desired
        Pose2d initPose; //4 different poses
        Pose2d midWayPose; //4 different poses
        Pose2d pickConePose; //4 different poses
        Pose2d dropCone; //4 different poses
        Pose2d parkPose; //4 different poses

        public void buildAuto(){
            if(Vision.playingAlliance == Vision.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                initPose = Vision.STARTPOS_1; //Starting pos when on blue alliance
                midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                dropCone =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones

            } else {
                initPose = Vision.STARTPOS_2; //Starting pos when on red alliance
                midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards cone
                pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                dropCone =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones

            }

            //Pick 5 cones and park
            trajInitToPickAndDropConeToPark = driveTrain.trajectorySequenceBuilder(initPose)
                    .lineToLinearHeading(midWayPose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,() -> {
                        turnToPole();
                        dropCone();
                    })
                    .lineToLinearHeading(dropCone)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,() -> {
                        turnToPole();
                        dropCone();
                    })
                    .lineToLinearHeading(dropCone)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,() -> {
                        turnToPole();
                        dropCone();
                    })
                    .lineToLinearHeading(dropCone)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,() -> {
                        turnToPole();
                        dropCone();
                    })
                    .lineToLinearHeading(dropCone)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,() -> {
                        turnToPole();
                        dropCone();
                    })
                    .lineToLinearHeading(dropCone)
                    .lineToLinearHeading(parkPose)
                    .build();
        }

        public void runAuto(){
            //Write any other actions to take during auto, or any other conditions for maneuvering
            driveTrain.followTrajectorySequence(trajInitToPickAndDropConeToPark);
            return;
        }

        //Write a method which is able to pick the cone depending on your subsystems
        public void pickCone(){

        }

        //Write a method which is able to turn to the pole depending on your subsystems
        public void turnToPole(){

        }

        //Write a method which is able to drop the cone depending on your subsystems
        public void dropCone(){

        }
        /**
         * Safe method to wait so that stop button is also not missed
         * @param time time in ms to wait
         */
        public void safeWait(double time){
            ElapsedTime timer = new ElapsedTime(MILLISECONDS);
            timer.reset();
            while (!isStopRequested() && timer.time() < time){
                //autonomousController.runAutoControl();
                driveTrain.update();
            }
        }

        public void loopWait(double time){
            ElapsedTime timer = new ElapsedTime(MILLISECONDS);
            timer.reset();
            while (!isStopRequested() && timer.time() < time){
                //autonomousController.runAutoControl();
                driveTrain.update();
            }
        }

        public void selectGamePlan() {
            telemetry.setAutoClear(true);
            telemetry.addData("Compile time : ", "22:00 :: 1/27/2022");

            //***** Select Alliance ******
            telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
            telemetry.update();

            while(!isStopRequested()){
                if(gamepad1.x){
                    Vision.playingAlliance = Vision.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                    Vision.ALLIANCE_FACTOR = -1;
                    telemetry.addData("Playing Alliance Selected : ", Vision.playingAlliance);
                    break;
                }
                if(gamepad1.b){
                    Vision.playingAlliance = Vision.PLAYING_ALLIANCE.RED_ALLIANCE;
                    Vision.ALLIANCE_FACTOR = 1;
                    telemetry.addData("Playing Alliance Selected : ", Vision.playingAlliance);
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            safeWait(200);

            //******select start pose******
            while(!isStopRequested()){
                telemetry.addData("Enter a Start Pose:","");
                telemetry.addData("Blue Left: (X)", "");
                telemetry.addData("Blue Right: (Y)", "");
                telemetry.addData("Red Left: (B)", "");
                telemetry.addData("Red Right: (A)", "");
                telemetry.addData("Playing Alliance Selected: ", Vision.playingAlliance);
                if(gamepad1.x){
                    Vision.startPosition = Vision.START_POSITION.POS1;
                    telemetry.addData("Start Position: ", Vision.startPosition);
                    break;
                }
                if(gamepad1.y){
                    Vision.startPosition = Vision.START_POSITION.POS2;
                    telemetry.addData("Start Position: ", Vision.startPosition);
                    break;
                }
                if(gamepad1.b){
                    Vision.startPosition = Vision.START_POSITION.POS3;
                    telemetry.addData("Start Position: ", Vision.startPosition);
                    break;
                }
                if(gamepad1.a){
                    Vision.startPosition = Vision.START_POSITION.POS4;
                    telemetry.addData("Start Position: ", Vision.startPosition);
                    break;
                }
                telemetry.update();
            }
            telemetry.addData("Playing Alliance Selected : ", Vision.playingAlliance);
            telemetry.addData("StartPose : ", Vision.startPosition);

            safeWait(200);

            //******select start pose******
            while(!isStopRequested()){
                telemetry.addData("Enter a Park Pose:","");
                telemetry.addData("Location 1: (X)", "");
                telemetry.addData("Location 2: (Y)", "");
                telemetry.addData("Location 3: (B)", "");
                telemetry.addData("Playing Alliance Selected: ", Vision.playingAlliance);
                if(gamepad1.x){
                    Vision.parkingLocation = Vision.PARKING_LOCATION.PARKPOS1;
                    telemetry.addData("Park Position: ", Vision.parkingLocation);
                    break;
                }
                if(gamepad1.y){
                    Vision.parkingLocation = Vision.PARKING_LOCATION.PARKPOS2;
                    telemetry.addData("Park Position: ", Vision.parkingLocation);
                    break;
                }
                if(gamepad1.b){
                    Vision.parkingLocation = Vision.PARKING_LOCATION.PARKPOS3;
                    telemetry.addData("Park Position: ", Vision.parkingLocation);
                    break;
                }
                telemetry.update();
            }
            telemetry.addData("Playing Alliance Selected : ", Vision.playingAlliance);
            telemetry.addData("StartPose : ", Vision.startPosition);
            telemetry.addData("ParkPose : ", Vision.parkingLocation);
        }

        /**
         * Method to add debug messages. Update as telemetry.addData.
         * Use public attributes or methods if needs to be called here.
         */
        public void printDebugMessages(){
            telemetry.setAutoClear(true);
            telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);


            telemetry.addData("Playing Alliance Selected : ", vision.playingAlliance);
            telemetry.addData("Start Position : ", Vision.startPosition);
            telemetry.addData("Parking Location : ", parkingLocation);


            telemetry.addData("Vision.playingAlliance : ", vision.playingAlliance);
            telemetry.addData("Vision.poseSetInAutonomous : ", vision.poseSetInAutonomous);
            telemetry.addData("Vision.currentPose : ", vision.currentPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            //telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

            telemetry.addData("Vision detectedLabel", vision.detectedLabel);
            telemetry.addData("Vision targetZone :", targetZone);
            telemetry.addData("Vision targetZoneLevel :", targetZoneLevel);

            telemetry.addData("Game Timer : ", gameTimer.time());

            telemetry.update();

        }
        }

