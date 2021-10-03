package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.AutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


/**
 * Ultimate Goal Autonomous mode <BR>
 *
 * This code describes how Autonomous mode is done by Hazmat Robot for Ultimate Goal.<BR>
 * The following options are coded here, and selectable through Gamepad inputs to set up <BR>
 *     <emsp>Playing Alliance : Red or Blue</emsp>
 *     <emsp>Start Line : Inner or Outer</emsp>
 *     <emsp>Game options :</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch, move Wobble Goal2 and park</emsp>
 *
 * The code for Red and Blue are written as reflection of each other.<BR>
 * Camera on either side is used using Vuforia to determine target for Wobble Goal<BR>
 */
//TODO: Copy and Rename Autonomous Mode
@Autonomous(name = "Hazmat Platform Autonomous", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp")
public class AutonomousOpModeTemplate extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public AutonomousController autonomousController;
    public DriveTrain driveTrain;
    public SubsystemTemplate subsystemTemplate;
    //TODO: Replace name of Subsystem1 and Declare more subsystems

    public Vision vision;
    public Pose2d startPose = GameField.BLUE_STARTPOS_1;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
    public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;

    double af = GameField.ALLIANCE_FACTOR;

    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        subsystemTemplate = new SubsystemTemplate(hardwareMap);
        //TODO: Replace name of Subsystem1 and Declare more subsystems

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1,gamepad2, driveTrain, subsystemTemplate);
        autonomousController = new AutonomousController(driveTrain, subsystemTemplate);

        //Key Pay inputs to select Game Plan;
        selectGamePlan(); //TODO: Update function with more selections as needed
        vision = new Vision(hardwareMap, activeWebcam);
        af = GameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        driveTrain.getLocalizer().setPoseEstimate(startPose);

        // Add logic to set starting state of Robot and hold at that
        /* example - Intake.setIntakeReleaseHold();

        while (MagazineUltimateGoal.magazineLaunchTouchSensor.isPressed() == false) {
            AutonomousController.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

         */

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = vision.runVuforiaTensorFlow();

            if (!parked){
                autonomousController.runAutoControl();
            }

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                vision.deactivateVuforiaTensorFlow();

                // Logic to determine and run defined Autonomous mode
                if (GameField.startPosition == GameField.START_POSITION.STARTPOS_1) {
                    runAutoOption1();
                    //TODO: Update Option1 with relevant name
                } else { //GameField.startPosition == GameField.START_POSITION.OUTER
                    //TODO: Create new runAutoOptions for alternate options
                    //runAutoOuter();
                }

                subsystemTemplate.setIntakeReleaseOpen();

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

        //Write last position to static class to be used as initial position in TeleOp
        GameField.currentPose = driveTrain.getPoseEstimate();
        GameField.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoOption1(){
        //TODO: Update name for Autonomous mode.
        //Logic for waiting
        safeWait(100);

        //TODO: Add logic for autonmous mode actions.

        /* Example

        // Move to launch position and launch rings to High Goal or Powershots
        if (!AutonomousController.launchHighGoalOrPowerShot) {
            //runInnerOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (AutonomousController.autoLaunchAim == AutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                AutonomousController.setMagazineToLaunch();
                AutonomousController.setLaunchTargetHighGoal();

                AutonomousController.setMagazineToLaunch();
                //Move to position to launch rings
                if (GameFieldUltimateGoal.playingAlliance == GameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = Drive.trajectoryBuilder(Drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                }
                Drive.followTrajectory(traj);

                traj = Drive.trajectoryBuilder(Drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-20, af * 48), Math.toRadians(af * 0))
                .splineToConstantHeading(new Vector2d(-33, af * 48), Math.toRadians(af * 0))
                .build();
                Drive.followTrajectory(traj);

                launch3RingsToHighGoal();
            }
            */
    }

    //TODO: Add other runAutoOptions


    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            autonomousController.runAutoControl();
        }
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:47 :: 2/13");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();

        //Add logic to select autonomous mode based on keypad entry
        while (!isStopRequested()) {
            if (gamepadController.gp1GetButtonBPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                GameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (gamepadController.gp1GetButtonXPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                GameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }
        telemetry.update();
        safeWait(200);

        //TODO: Update instructions per game positions
        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(STARTPOS_1: (A) ,    STARTPOS_2: (Y))");
        while (!isStopRequested()) {
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (gamepadController.gp1GetButtonAPress()) {
                    GameField.startPosition = GameField.START_POSITION.STARTPOS_1;
                    startPose = GameField.RED_STARTPOS_1;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "RED_STARTPOS_1");
                    break;
                }
                if (gamepadController.gp1GetButtonYPress()) {
                    GameField.startPosition = GameField.START_POSITION.STARTPOS_2;
                    startPose = GameField.RED_STARTPOS_2;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "RED_STARTPOS_2");
                    break;
                }
            }
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (gamepadController.gp1GetButtonAPress()) {
                    GameField.startPosition = GameField.START_POSITION.STARTPOS_1;
                    startPose = GameField.BLUE_STARTPOS_1;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "BLUE_STARTPOS_1");
                    break;
                }
                if (gamepadController.gp1GetButtonYPress()) {
                    GameField.startPosition = GameField.START_POSITION.STARTPOS_2;
                    startPose = GameField.BLUE_STARTPOS_2;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "BLUE_STARTPOS_2");
                    break;
                }
            }
            telemetry.update();
        }

        //TODO: Add more selection logic based on the above template

        telemetry.update();
        safeWait(200);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

        //****** Vision Debug *****
        telemetry.addData("Target Label : ", vision.detectedLabel);
        telemetry.addData("Target Left : ", vision.detectedLabelLeft);
        telemetry.addData("Target Right : ", vision.detectedLabelRight);
        telemetry.addData("Target Top : ", vision.detectedLabelTop);
        telemetry.addData("Target Bottom : ", vision.detectedLabelBottom);

        //TODO:Add logic for debug print Logic


        telemetry.update();

    }
}

