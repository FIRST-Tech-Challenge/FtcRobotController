package org.firstinspires.ftc.teamcode.AutoOpMode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
        public Vision.PARKING_LOCATION parkingLocation = Vision.PARKING_LOCATION.ENDPOS1;
        public int loopPickConetoDrop = 0;
        public boolean pickCone = false;

        boolean parked = false ;
        public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

        public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
        public Vision.VISION_IDENTIFIED_TARGET targetZone = Vision.VISION_IDENTIFIED_TARGET.RED;//Set a default vision value
        int targetZoneLevel = 0;

        double af = Vision.ALLIANCE_FACTOR;

        Trajectory traj;
        TrajectorySequence trajSeq;

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
        TrajectorySequence trajInitMoveAndTurnToConePosition;
        TrajectorySequence trajConeToPark;
        TrajectorySequence[] pickToTurnToDropCone = new TrajectorySequence[5];

        //Initialize any other Pose2d's as desired
        Pose2d initPose; //4 different poses
        Pose2d offWallPose; //4 different poses
        Pose2d conePose; //4 different poses
        Pose2d parkPose; //4 different poses

        public void buildAuto(){
            if(Vision.playingAlliance == Vision.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                initPose = Vision.STARTPOS_1; //Starting pos when on blue alliance
                offWallPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                conePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                //Initialize any other Pose2ds needed for maneuvering through the field in auto
            } else {
                initPose = Vision.STARTPOS_2; //Starting pos when on red alliance
                offWallPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards cone
                conePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                //Initialize any other Pose2ds needed for maneuvering through the field in auto
            }

            //Sequence to move from initPose to
            trajInitMoveAndTurnToConePosition = driveTrain.trajectorySequenceBuilder(initPose)
                    .lineToLinearHeading(offWallPose)
                    //Add any subsystems actions here as well
                    //Add any other steps to take in order to move from init pose to cone detection pose
                    .build();

            //Loops to pick and drop cones
            if(Vision.playingAlliance == Vision.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                if(initPose == Vision.STARTPOS_1){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else if(initPose == Vision.STARTPOS_2){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else if(initPose == Vision.STARTPOS_3){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else {//Pose -> STARTPOS_4
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                }

            } else {//red alliance start
                if(initPose == Vision.STARTPOS_1){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else if(initPose == Vision.STARTPOS_2){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else if(initPose == Vision.STARTPOS_3){
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                } else {//Pose -> STARTPOS_4
                    for (int i = 0; i < 5; i++) {
                        pickToTurnToDropCone[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                                .lineToLinearHeading(conePose)
                                //Add any subsystems actions here as well
                                //Add any other steps to take in order to move from offWallPose->pickConePose->dropPose->pickConePose
                                .build();
                    }
                }
            }

            if(Vision.playingAlliance == Vision.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                if(Vision.startPosition == Vision.START_POSITION.POS1){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            //Add any subsystems actions here as well
                            //Add any other steps to take in order to move
                            .build();
                } else if(Vision.startPosition == Vision.START_POSITION.POS2){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            //Add any subsystems actions here as well
                            //Add any other steps to take in order to move
                            .build();
                } else if(Vision.startPosition == Vision.START_POSITION.POS3){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            //Add any subsystems actions here as well
                            //Add any other steps to take in order to move to drop
                            .build();
                } else { //POS4
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            //Add any subsystems actions here as well
                            //Add any other steps to take in order to move
                            .build();
                }
            } else {//Red Alliance
                if(Vision.startPosition == Vision.START_POSITION.POS1){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            .build();
                } else if(Vision.startPosition == Vision.START_POSITION.POS2){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            .build();
                } else if(Vision.startPosition == Vision.START_POSITION.POS3){
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            .build();
                } else { //POS4
                    trajConeToPark = driveTrain.trajectorySequenceBuilder(conePose)
                            .lineToLinearHeading(parkPose)
                            .build();
                }
            }

        }

        public void runAuto(){
            //Write any other actions to take during auto, or any other conditions for maneuvering
            driveTrain.followTrajectorySequence(trajInitMoveAndTurnToConePosition);
            pickCone();
            for (int loop = 0; loop < 5; loop++) {
                driveTrain.followTrajectorySequence(pickToTurnToDropCone[loop]);
                safeWait(100);
            }
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

            //Add logic to select autonomous mode based on keypad entry
            /*
            while (!isStopRequested()) {

                if (gamepadController.gp1GetButtonBPress()) {
                    GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                    GameField.ALLIANCE_FACTOR = -1;
                    telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                    break;
                }
                if (gamepadController.gp1GetButtonXPress()) {
                    GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                    GameField.ALLIANCE_FACTOR = 1;
                    telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            safeWait(200);
                 */
            }

        /**
         * Method to add debug messages. Update as telemetry.addData.
         * Use public attributes or methods if needs to be called here.
         */
        public void printDebugMessages(){
            telemetry.setAutoClear(true);
            telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);


            telemetry.addData("Playing Alliance Selected : ", vision.playingAlliance);
            telemetry.addData("Start Position : ", vision.startPosition);
            telemetry.addData("Parking Location : ", parkingLocation);


            telemetry.addData("GameField.playingAlliance : ", vision.playingAlliance);
            telemetry.addData("GameField.poseSetInAutonomous : ", vision.poseSetInAutonomous);
            telemetry.addData("GameField.currentPose : ", vision.currentPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            //telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

            telemetry.addData("Vision targetLevelDetected : ", vision.targetLevelDetected);
            telemetry.addData("Vision detectedLabel", vision.detectedLabel);
            telemetry.addData("Vision detectedLabelLeft :", vision.detectedLabelLeft);
            telemetry.addData("Vision targetZone :", targetZone);
            telemetry.addData("Vision targetZoneLevel :", targetZoneLevel);

            telemetry.addData("Game Timer : ", gameTimer.time());

            telemetry.update();

        }
        }

