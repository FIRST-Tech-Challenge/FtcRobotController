package org.firstinspires.ftc.teamcode.GameOpModes.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzGamepadControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzLaunchSubControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzVuforiaUltimateGoal;


/**
 * Ultimate Goal TeleOp mode with Vuforia Odometry correction <BR>
 *
 *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *  This code uses Vuforia additional to roadrunner to correct for Odometry errors <BR>
 *
 */
@Disabled
@TeleOp(name = "HzTeleOp RR Viewforia Ultimate Goal", group = "00-Teleop")
public class HzTeleOpRRVuforiaUltimateGoal extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadControllerUltimateGoal hzGamepadControllerUltimateGoal;
    public DriveTrain driveTrain;
    public HzMagazineUltimateGoal hzMagazineUltimateGoal;
    public HzIntakeUltimateGoal hzIntakeUltimateGoal;
    public HzLaunchSubControllerUltimateGoal hzLaunchSubControllerUltimateGoal;
    public HzLauncherUltimateGoal hzLauncherUltimateGoal;
    public HzArmUltimateGoal hzArmUltimateGoal;

    public HzVuforiaUltimateGoal hzVuforiaUltimateGoal;
    public Pose2d startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
    public HzVuforiaUltimateGoal.ACTIVE_WEBCAM activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
    //int playingAlliance = 0; //1 for Red, -1 for Blue, 0 for Audience
    //TODO : Create another TeleOp for Red

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        driveTrain = new DriveTrain(hardwareMap);
        hzMagazineUltimateGoal = new HzMagazineUltimateGoal(hardwareMap);
        hzIntakeUltimateGoal = new HzIntakeUltimateGoal(hardwareMap);

        hzLauncherUltimateGoal = new HzLauncherUltimateGoal(hardwareMap);
        hzArmUltimateGoal = new HzArmUltimateGoal(hardwareMap);
        hzLaunchSubControllerUltimateGoal = new HzLaunchSubControllerUltimateGoal(hardwareMap, hzLauncherUltimateGoal, hzIntakeUltimateGoal, hzMagazineUltimateGoal, driveTrain);
        hzGamepadControllerUltimateGoal = new HzGamepadControllerUltimateGoal(gamepad1, driveTrain, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);

        initialConfiguration();

        hzVuforiaUltimateGoal = new HzVuforiaUltimateGoal(hardwareMap,activeWebcam);
        hzVuforiaUltimateGoal.setupVuforiaNavigation();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //hzDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO : When implementing Autonomous Mode uncomment this section.
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //hzDrive.setPoseEstimate(PoseStorage.currentPose);
        if ( HzGameFieldUltimateGoal.poseSetInAutonomous == true) {
            driveTrain.getLocalizer().setPoseEstimate(HzGameFieldUltimateGoal.currentPose);
        } else {
            driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        //TODO : IF PROGRAM CRASHES GO MANUAL ALL THE TIME

        //Activate Vuforia Navigation
        //TODO : ACTIVATE VUFORIA NAVIGATION ON A THREAD SO THAT WAIT DOES NOT HAPPEN
        hzVuforiaUltimateGoal.activateVuforiaNavigation();

        //Run Vuforia Navigation
        hzVuforiaUltimateGoal.runVuforiaNavigation();

        // Initiate Camera even before Start is pressed.
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //TODO : IS THIS THE REASON FOR CRASH WITHOUT STOP BEING HANDLED
            //Init is pressed at this time, and start is not pressed yet

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {

                hzVuforiaUltimateGoal.runVuforiaNavigation();

                hzGamepadControllerUltimateGoal.runByGamepadControl();

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        hzVuforiaUltimateGoal.deactivateVuforiaNavigation();
        HzGameFieldUltimateGoal.poseSetInAutonomous = false;
    }


    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X)");
        telemetry.update();

        while (!isStopRequested()) {
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            //telemetry.addData("10s time : Default Alliance A :",);
            telemetry.update();
        }

        telemetry.update();
        sleep(500);

        //***** Select Start Pose ******
        //timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (!isStopRequested()) {
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            //telemetry.addData("10s Timer : Default Pose : BLUE_INNER_START_LINE : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();
        sleep(500);
    }


    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", HzGameFieldUltimateGoal.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", HzGameFieldUltimateGoal.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", HzGameFieldUltimateGoal.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("hzDrive.driveMode : ", driveTrain.driveMode);
        telemetry.addData("hzDrive.poseEstimate :", driveTrain.poseEstimate);

        // Print pose to telemetry
        telemetry.addData("hzVuforia1.visibleTargetName : ", hzVuforiaUltimateGoal.visibleTargetName);
        telemetry.addData("hzVuforia1.poseVuforia :", hzVuforiaUltimateGoal.poseVuforia);
        //telemetry.addData("hzVuforia.vuforiaFirstAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaFirstAngle));
        //telemetry.addData("hzVuforia.vuforiaSecondAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaSecondAngle));
        //telemetry.addData("hzVuforia.vuforiaThirdAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaThirdAngle));

        //******* Magazine Debug ********
        switch (hzMagazineUltimateGoal.getMagazinePosition()) {
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
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed());
        //telemetry.addData("hzMagazine.moveMagazineToCollectState",hzMagazine.moveMagazineToCollectState);
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazineUltimateGoal.magazineCollectTouchSensor.isPressed());


        //********** Intake Debug *******
        //telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepad.getDpad_downPress());
        //telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepad.getDpad_upPress());
        //telemetry.addData("intakeMotor.isBusy()", hzIntake.intakeMotor.isBusy());
        switch (hzIntakeUltimateGoal.getIntakeState()){
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
        telemetry.addData("hzLaunchController.launchMode : ", hzLaunchSubControllerUltimateGoal.launchMode);
        telemetry.addData("hzLaunchController.deactivateLaunchReadinessState :", hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState);
        telemetry.addData("hzLaunchController.activateLaunchReadinessState :", hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState);
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchSubControllerUltimateGoal.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchSubControllerUltimateGoal.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchSubControllerUltimateGoal.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchSubControllerUltimateGoal.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchSubControllerUltimateGoal.distanceFromTarget);
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchSubControllerUltimateGoal.lclaunchMotorPower);
        //telemetry.addData("hzLauncher.launchMotorVelocity : ", hzLauncher.launchMotorVelocity);
        telemetry.addData("hzDrive.drivePointToAlign : ", driveTrain.drivePointToAlign);

        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncherUltimateGoal.launcherRingPlungerServo.getPosition());

        switch (hzLauncherUltimateGoal.getLauncherState()){
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
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

        switch (hzArmUltimateGoal.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());

        //telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());
        //telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

        switch (hzArmUltimateGoal.getCurrentArmPosition()){
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
