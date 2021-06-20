package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.HzArm;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzGameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzIntake;
import org.firstinspires.ftc.teamcode.SubSystems.HzLaunchController;
import org.firstinspires.ftc.teamcode.SubSystems.HzLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.HzMagazine;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Hazmat TeleOp RR", group = "00-Teleop")
public class HzTeleOpRR extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    public HzDrive hzDrive;
    public HzMagazine hzMagazine;
    public HzIntake hzIntake;

    public HzLaunchController hzLaunchController;
    public HzLauncher hzLauncher;
    public HzArm hzArm;

    //public HzVuforia hzVuforia1;
    public Pose2d startPose = HzGameField.calibPoint;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new HzMagazine(hardwareMap);
        hzIntake = new HzIntake(hardwareMap);

        hzLauncher = new HzLauncher(hardwareMap);
        hzArm = new HzArm(hardwareMap);
        hzLaunchController = new HzLaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        if ( HzGameField.poseSetInAutonomous == true) {
            hzDrive.getLocalizer().setPoseEstimate(HzGameField.currentPose);
        } else {
            hzDrive.getLocalizer().setPoseEstimate(startPose);
        }

        //TODO : IF PROGRAM CRASHES GO MANUAL ALL THE TIME
        hzIntake.setIntakeReleaseOpen();

        // Initiate Camera even before Start is pressed.
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Activate Vuforia Navigation
            //hzVuforia1.activateVuforiaNavigation();

            //Run Vuforia Navigation
            //hzVuforia1.runVuforiaNavigation();

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {

                //hzVuforia1.runVuforiaNavigation();

                hzGamepad.runByGamepad();

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        //hzVuforia1.deactivateVuforiaNavigation();
        HzGameField.poseSetInAutonomous = false;
    }

    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X)");
        telemetry.update();

        while (!isStopRequested()) {
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        sleep(500);

        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (!isStopRequested()) {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.RED_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.RED_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.BLUE_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.BLUE_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
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
