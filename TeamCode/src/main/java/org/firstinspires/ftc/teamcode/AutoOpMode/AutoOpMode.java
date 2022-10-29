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

        TrajectorySequence[] trajOffWallTurnToCone = new TrajectorySequence[3];
        //TrajectorySequence[] pickToTurnToDropCone = new TrajectorySequence[];

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

