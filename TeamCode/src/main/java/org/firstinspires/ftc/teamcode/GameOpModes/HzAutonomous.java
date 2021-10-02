package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.HzAutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.HzGamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzSubsystem1;
import org.firstinspires.ftc.teamcode.SubSystems.HzVision;

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
public class HzAutonomous extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadController hzGamepadController;
    public HzAutonomousController hzAutonomousController;
    public HzDrive hzDrive;
    public HzSubsystem1 hzSubsystem1;
    //TODO: Replace name of Subsystem1 and Declare more subsystems

    public HzVision hzVision;
    public Pose2d startPose = HzGameField.BLUE_STARTPOS_1;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzVision.ACTIVE_WEBCAM activeWebcam = HzVision.ACTIVE_WEBCAM.WEBCAM1;
    public HzGameField.VISION_IDENTIFIED_TARGET targetZone = HzGameField.VISION_IDENTIFIED_TARGET.LEVEL1;

    double af = HzGameField.ALLIANCE_FACTOR;

    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        hzDrive = new HzDrive(hardwareMap);
        hzSubsystem1 = new HzSubsystem1(hardwareMap);
        //TODO: Replace name of Subsystem1 and Declare more subsystems

        /* Create Controllers */
        hzGamepadController = new HzGamepadController(gamepad1,gamepad2, hzDrive, hzSubsystem1);
        hzAutonomousController = new HzAutonomousController(hzDrive, hzSubsystem1);

        //Key Pay inputs to select Game Plan;
        selectGamePlan(); //TODO: Update function with more selections as needed
        hzVision = new HzVision(hardwareMap, activeWebcam);
        af = HzGameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVision.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        // Add logic to set starting state of Robot and hold at that
        /* example - hzIntake.setIntakeReleaseHold();

        while (hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutonomousController.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

         */

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = hzVision.runVuforiaTensorFlow();

            if (!parked){
                hzAutonomousController.runAutoControl();
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVision.deactivateVuforiaTensorFlow();

                // Logic to determine and run defined Autonomous mode
                if (HzGameField.startPosition == HzGameField.START_POSITION.STARTPOS_1) {
                    runAutoOption1();
                    //TODO: Update Option1 with relevant name
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
                    //TODO: Create new runAutoOptions for alternate options
                    //runAutoOuter();
                }

                hzSubsystem1.setIntakeReleaseOpen();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                HzGameField.currentPose = hzDrive.getPoseEstimate();
                HzGameField.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        HzGameField.currentPose = hzDrive.getPoseEstimate();
        HzGameField.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoOption1(){
        //TODO: Update name for Autonomous mode.
        //Logic for waiting
        hzWait(100);

        //TODO: Add logic for autonmous mode actions.

        /* Example

        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutonomousController.launchHighGoalOrPowerShot) {
            //runInnerOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

                hzAutonomousController.setMagazineToLaunch();
                //Move to position to launch rings
                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                }
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-20, af * 48), Math.toRadians(af * 0))
                .splineToConstantHeading(new Vector2d(-33, af * 48), Math.toRadians(af * 0))
                .build();
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();
            }
            */
    }

    //TODO: Add other runAutoOptions


    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutonomousController.runAutoControl();
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
            if (hzGamepadController.gp1GetButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadController.gp1GetButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }
        telemetry.update();
        hzWait(200);

        //TODO: Update instructions per game positions
        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(STARTPOS_1: (A) ,    STARTPOS_2: (Y))");
        while (!isStopRequested()) {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepadController.gp1GetButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.STARTPOS_1;
                    startPose = HzGameField.RED_STARTPOS_1;
                    activeWebcam = HzVision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "RED_STARTPOS_1");
                    break;
                }
                if (hzGamepadController.gp1GetButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.STARTPOS_2;
                    startPose = HzGameField.RED_STARTPOS_2;
                    activeWebcam = HzVision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "RED_STARTPOS_2");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepadController.gp1GetButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.STARTPOS_1;
                    startPose = HzGameField.BLUE_STARTPOS_1;
                    activeWebcam = HzVision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "BLUE_STARTPOS_1");
                    break;
                }
                if (hzGamepadController.gp1GetButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.STARTPOS_2;
                    startPose = HzGameField.BLUE_STARTPOS_2;
                    activeWebcam = HzVision.ACTIVE_WEBCAM.WEBCAM1;
                    telemetry.addData("Start Pose : ", "BLUE_STARTPOS_2");
                    break;
                }
            }
            telemetry.update();
        }

        //TODO: Add more selection logic based on the above template

        telemetry.update();
        hzWait(200);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", HzGameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", HzGameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", HzGameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);
        telemetry.addData("Battery Power", hzDrive.getBatteryVoltage(hardwareMap));

        //****** Vision Debug *****
        telemetry.addData("Target Label : ", hzVision.detectedLabel);
        telemetry.addData("Target Left : ", hzVision.detectedLabelLeft);
        telemetry.addData("Target Right : ", hzVision.detectedLabelRight);
        telemetry.addData("Target Top : ", hzVision.detectedLabelTop);
        telemetry.addData("Target Bottom : ", hzVision.detectedLabelBottom);

        //TODO:Add logic for debug print Logic


        telemetry.update();

    }
}

