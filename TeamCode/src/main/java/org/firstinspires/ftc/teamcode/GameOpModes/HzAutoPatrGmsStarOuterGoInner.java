package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.HzArm;
import org.firstinspires.ftc.teamcode.SubSystems.HzAutoControl;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzGameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzIntake;
import org.firstinspires.ftc.teamcode.SubSystems.HzLaunchController;
import org.firstinspires.ftc.teamcode.SubSystems.HzLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.HzMagazine;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


/**
 * Ultimate Goal Autonomous mode <BR>
 *
 * This code describes how Autonomous mode is done by Hazmat Robot for Ultimate Goal.<BR>
 * The following options are coded here, and selectable through Gamepad inputs to set up <BR>
 *     <emsp>Playing Alliance : Red or Blue</emsp>
 *     <emsp>Start Line : Inner or Outer</emsp>
 *     <emsp>Game options :</emsp>
 *     <emsp>   Launch rings to Power Shot of High Goal </emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch, move Wobble Goal2 and park</emsp>
 *
 * The code for Red and Blue are written as reflection of each other.<BR>
 * Camera on either side is used using Vuforia to determine target for Wobble Goal<BR>
 */
@Autonomous(name = "Hazmat XXX StartOuterGoInner", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
public class HzAutoPatrGmsStarOuterGoInner extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    public HzAutoControl hzAutoControl;
    public HzDrive hzDrive;
    public HzMagazine hzMagazine;
    public HzIntake hzIntake;
    public HzLaunchController hzLaunchController;
    public HzLauncher hzLauncher;
    public HzArm hzArm;

    public HzVuforia hzVuforia;
    public Pose2d startPose = HzGameField.BLUE_INNER_START_LINE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzGameField.TARGET_ZONE targetZone = HzGameField.TARGET_ZONE.A;
    public HzVuforia.ACTIVE_WEBCAM activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;

    double af = HzGameField.ALLIANCE_FACTOR;

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize HW
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new HzMagazine(hardwareMap);
        hzIntake = new HzIntake(hardwareMap);

        hzLauncher = new HzLauncher(hardwareMap);
        hzArm = new HzArm(hardwareMap);
        hzLaunchController = new HzLaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);
        hzAutoControl = new HzAutoControl(hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
        hzVuforia = new HzVuforia(hardwareMap, activeWebcam);
        af = HzGameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVuforia.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntake.setIntakeReleaseHold();
        hzAutoControl.setMagazineToLaunch();

        while (hzMagazine.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutoControl.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = hzVuforia.runVuforiaTensorFlow();

            if (!parked){
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.runAutoControl();
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchController.launchMode = HzLaunchController.LAUNCH_MODE.MANUAL;

            hzLauncher.flyWheelVelocityHighGoal = 1430;
            hzLauncher.flyWheelVelocityPowerShot = 1330;

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforia.deactivateVuforiaTensorFlow();

                runAutoStartOuterGoInner();

                hzIntake.setIntakeReleaseOpen();
                hzAutoControl.setMagazineToCollect();

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




    public void runAutoStartOuterGoInner(){
        //Assume Start Outer
        if (!hzAutoControl.launchHighGoalOrPowerShot) {
            hzWait(12000);
        } else {
            hzWait(8000);
        }

        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-53, af *10, Math.toRadians(af * -90))) //-40,6
                .build();
        hzDrive.followTrajectory(traj);

        if (!hzAutoControl.launchHighGoalOrPowerShot) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 6, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.setLaunchTargetHighGoal();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    if (!hzAutoControl.dropFirstWobbleGoal) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-54, af *10, Math.toRadians(af * 90))) //-40,6
                                .build();
                        hzDrive.followTrajectory(traj);
                    } else {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                }
                hzAutoControl.setMagazineToLaunch();
                //Move to position to launch rings
                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, 14, Math.toRadians(19)))//-10  ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, -14, Math.toRadians(-12)))//-10 ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-13, -12, Math.toRadians(-13))) //STATE TESTING
                            .build();
                }
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutoControl.setLaunchTargetPowerShot1();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                //Move to position to launch rings

                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 5)))
                            .lineToLinearHeading(new Pose2d(-10, af * 19, Math.toRadians(af * 5)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                } else {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                }
                launch3RingsToPowerShots();

            }
        }

        hzWait(2000);
        //Park
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(13, af * 6, Math.toRadians(af * 0)))
                .build();
        hzDrive.followTrajectory(traj);

    }

    /**
     * Sequence of launching 3 rings to High Goal
     */
    public void launch3RingsToHighGoal(){
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400); //400
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setRunLauncherTrue();

        // Run 4th just in case
        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
        }
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to PowerShots
     */
    public void launch3RingsToPowerShots(){
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(600);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot12);
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot23);
        hzAutoControl.setMagazineToLaunch();
        hzAutoControl.setLaunchTargetPowerShot1();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();

    }

    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutoControl.runAutoControl();
        }
    }

    /**
     * Method that take key pad inputs to select the Autonomous options
     */
    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:47 :: 2/13");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();


        while (!isStopRequested()) {
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        hzWait(200);

        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(Inner: (A) ,    Outer: (Y))");
        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
            HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
            startPose = HzGameField.RED_OUTER_START_LINE;
            activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
            telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
        } else {
            HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
            startPose = HzGameField.BLUE_OUTER_START_LINE;
            activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
            telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
        }
        telemetry.update();

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "");
            telemetry.addData("     (X) for Powershot","");
            telemetry.addData("     (B) for High Goal","");
            telemetry.addData("     (A) for No launch - Only park","");
            if (hzGamepad.getButtonBPress()) {
                hzAutoControl.launchHighGoalOrPowerShot = true;
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.launchHighGoalOrPowerShot = true;
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonAPress()) {
                /*hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);*/
                telemetry.addData("No Launch, Only Park : ","");
                hzAutoControl.launchHighGoalOrPowerShot = false;
                break;
            }
            telemetry.update();
            //hzWait(200);
        }

        hzAutoControl.dropFirstWobbleGoal = false;
        hzAutoControl.pickRingFromTargetMarker = false;
        hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
        hzAutoControl.pickAndDropSecondWobbleGoal = false;
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

        //telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia :",hzVuforia1.poseVuforia);

        telemetry.addData("targetZone identified: ", targetZone);

        //******* Magazine Debug ********
        switch (hzMagazine.getMagazinePosition()) {
            case AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition(): ", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_COLLECT");
                break;
            }
            case AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_ERROR");
                break;
            }
        }
        //telemetry.addData("hzMagazine.moveMagazineToLaunchState",hzMagazine.moveMagazineToLaunchState);
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
        //telemetry.addData("hzMagazine.moveMagazineToCollectState",hzMagazine.moveMagazineToCollectState);
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazine.magazineCollectTouchSensor.isPressed());


        //********** Intake Debug *******
        switch (hzIntake.getIntakeState()){
            case RUNNING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_RUNNING");
                break;
            }
            case STOPPED: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_STOPPED");
                break;
            }
            case REVERSING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_REVERSING");
                break;
            }
        }

        //******* Launch Controller Debug ********
        telemetry.addData("hzLaunchController.launchMode : ", hzLaunchController.launchMode);
        telemetry.addData("hzLaunchController.deactivateLaunchReadinessState :",hzLaunchController.deactivateLaunchReadinessState);
        telemetry.addData("hzLaunchController.activateLaunchReadinessState :",hzLaunchController.activateLaunchReadinessState);
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchController.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchController.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchController.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchController.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchController.distanceFromTarget);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getPower() : ", hzLauncher.launcherFlyWheelMotor.getPower());
        telemetry.addData("hzLaunchController.lclaunchMotorVelocity : ", hzLaunchController.lclaunchMotorVelocity);
        telemetry.addData("hzLauncher.flyWheelVelocityPowerShot : ", hzLauncher.flyWheelVelocityPowerShot);
        telemetry.addData("hzLauncher.flyWheelVelocityHighGoal : ", hzLauncher.flyWheelVelocityHighGoal);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getVelocity() : ", hzLauncher.launcherFlyWheelMotor.getVelocity());
        telemetry.addData("hzDrive.drivePointToAlign : ", hzDrive.drivePointToAlign);

        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncher.launcherRingPlungerServo.getPosition());

        switch (hzLauncher.getLauncherState()){
            case RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_TARGET");
                break;
            }
            case RUNNING_FOR_SUPPLY:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case STOPPED:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_STOPPED");
                break;
            }
        }




        //***** Arm Debug ****
        telemetry.addData("armMotor.baseline", hzArm.baselineEncoderCount);
        telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

        switch (hzArm.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        switch (hzArm.getCurrentArmPosition()){
            case PARKED: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PARKED");
                break;
            }
            case DROP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "DROP_WOBBLE_RING");
                break;
            }
            case HOLD_UP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "HOLD_UP_WOBBLE_RING");
                break;
            }
            case PICK_WOBBLE:{
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_WOBBLE");
                break;
            }
            case PICK_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_RING");
                break;
            }
        }

        telemetry.update();

    }
}

