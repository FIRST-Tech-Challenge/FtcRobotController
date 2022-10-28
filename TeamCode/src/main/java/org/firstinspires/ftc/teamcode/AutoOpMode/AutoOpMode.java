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
        //public Vision vision;

        public Pose2d startPose = GameField.BLUE_ALLIANCE_BLUE_TERMINAL;

        //need to fix gamefield parking positions
        //public GameField.PARKING_LOCATION parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
        public GameField.AUTONOMOUS_ROUTE autonomousRoute = GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER;
        public int loopsFromWarehouseToAlShippingHub = 0;
        public boolean whLoopParkThroughBarrier = false;
        public boolean pickShippingElement = false;

        boolean parked = false ;
        public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

        //public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
        public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;
        int targetZoneLevel = 0;

        double af = GameField.ALLIANCE_FACTOR;

        Trajectory traj;
        TrajectorySequence trajSeq;

        @Override
        public void runOpMode() throws InterruptedException {
            /*Create your Subsystem Objects*/
            driveTrain = new DriveTrain(hardwareMap);



            /* Create Controllers */

            //Key Pay inputs to select Game Plan;
            selectGamePlan();
            //vision = new Vision(hardwareMap, activeWebcam);
            af = GameField.ALLIANCE_FACTOR;

            // Initiate Camera on Init.
            //vision.activateVuforiaTensorFlow();

            /*
            if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
                buildAutoWarehouse();
            } else { //GameField.startPosition == GameField.START_POSITION.STORAGE
                buildAutoStorage();
            }
             */
            driveTrain.getLocalizer().setPoseEstimate(startPose);


           //autonomousController.runAutoControl();

            telemetry.addData("Waiting for start to be pressed.", "Robot is ready!");
            telemetry.update();

            if (isStopRequested()) return;

            while (!isStopRequested()) {

                //Run Vuforia Tensor Flow
                //targetZone = vision.runVuforiaTensorFlow();
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

                    //vision.deactivateVuforiaTensorFlow();

                    /*
                    autonomousController.moveAutoElevatorLevel1();
                    autonomousController.moveAutoMagazineToTransport();
                     */


                    // Logic to determine and run defined Autonomous mode
                    if (GameField.startPosition == GameField.START_POSITION.BLUE_TERMINAL) {
                       // runAutoWarehouse();
                    } else {
                       // runAutoStorage();
                    }


                    //Move to Launching Position
                    parked = true;

                    //Write last position to static class to be used as initial position in TeleOp
                    GameField.currentPose = driveTrain.getPoseEstimate();
                    GameField.poseSetInAutonomous = true;

                    if (DEBUG_FLAG) {
                        printDebugMessages();
                        telemetry.update();
                    }
                }

            }
            //autonomousController.moveAutoElevatorLevel0();
            safeWait(100);
            GameField.currentPose = driveTrain.getPoseEstimate();
            GameField.poseSetInAutonomous = true;
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

            telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
            telemetry.addData("Start Position : ", GameField.startPosition);
            telemetry.addData("Autonomous route : ", autonomousRoute);
            //telemetry.addData("Parking Location : ", parkingLocation);
            telemetry.addData("Pick Shipping Element : ", pickShippingElement);

            telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
            telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            telemetry.addData("startPose : ", startPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            //telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

            //telemetry.addData("Vision targetLevelDetected : ", vision.targetLevelDetected);
            //telemetry.addData("Vision detectedLabel", vision.detectedLabel);
            //telemetry.addData("Vision detectedLabelLeft :", vision.detectedLabelLeft);
            telemetry.addData("Vision targetZone :", targetZone);
            telemetry.addData("Vision targetZoneLevel :", targetZoneLevel);

            telemetry.addData("Game Timer : ", gameTimer.time());

            telemetry.update();

        }
        }

