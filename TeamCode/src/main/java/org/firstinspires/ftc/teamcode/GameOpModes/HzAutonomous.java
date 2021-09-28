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
@Autonomous(name = "Hazmat Platform Autonomous", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp")
public class HzAutonomous extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadController hzGamepadController;
    public HzAutonomousController hzAutonomousController;
    public HzDrive hzDrive;
    public HzSubsystem1 hzSubsystem1;

    public HzVision hzVision;
    public Pose2d startPose = HzGameField.BLUE_INNER_START_LINE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzVision.ACTIVE_WEBCAM activeWebcam = HzVision.ACTIVE_WEBCAM.LEFT;
    public HzGameField.TARGET_ZONE targetZone = HzGameField.TARGET_ZONE.A;

    double af = HzGameField.ALLIANCE_FACTOR;

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        hzDrive = new HzDrive(hardwareMap);
        hzSubsystem1 = new HzSubsystem1(hardwareMap);
        /* Create Controllers */
        hzGamepadController = new HzGamepadController(gamepad1,hzDrive, hzSubsystem1);
        hzAutonomousController = new HzAutonomousController(hzDrive, hzSubsystem1);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
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
                if (HzGameField.startPosition == HzGameField.START_POSITION.INNER) {
                    runAutoOption1();
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
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

        //Logic for waiting
        hzWait(100);

        //Add logic for autonmous mode actions.
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

        /* Example
        while (!isStopRequested()) {
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameFieldUltimateGoal.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameFieldUltimateGoal.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }
         */

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

        //Add logic for debug print Logic

        telemetry.update();

    }
}

