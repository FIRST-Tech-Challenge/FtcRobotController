package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchController;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;


/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "HzTeleOp RR Field-Viewforia", group = "00-Teleop")
public class HzTeleOpRRFieldCentricVuforia extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    //public GameField hzGameField;
    //public SampleMecanumDrive hzDrive;
    public HzDrive hzDrive;
    public Magazine hzMagazine;
    public Intake hzIntake;
    public LaunchController hzLaunchController;
    public Launcher hzLauncher;
    public Arm hzArm;

    public HzVuforia hzVuforia;
    public Pose2d startPose = GameField.ORIGIN_FIELD;
    //int playingAlliance = 0; //1 for Red, -1 for Blue, 0 for Audience
    //TODO : Create another TeleOp for Red

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new Magazine(hardwareMap);
        hzIntake = new Intake(hardwareMap);

        hzLauncher = new Launcher(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzLaunchController = new LaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        initialConfiguration();

        hzVuforia = new HzVuforia(hardwareMap);
        hzVuforia.setupVuforiaNavigation();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //hzDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO : When implementing Autonomous Mode uncomment this section.
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //hzDrive.setPoseEstimate(PoseStorage.currentPose);
        if ( PoseStorage.poseSetInAutonomous == true) {
            hzDrive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        } else {
            hzDrive.getLocalizer().setPoseEstimate(startPose);
        }

        //Activate Vuforia Navigation
        hzVuforia.activateVuforiaNavigation();

        //Run Vuforia Navigation
        hzVuforia.runVuforiaNavigation();

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

                hzVuforia.runVuforiaNavigation();

                hzGamepad.runByGamepad();

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        hzVuforia.deactivateVuforiaNavigation();
        PoseStorage.poseSetInAutonomous = false;
    }

    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        telemetry.addData("Compile time : ", "6:50 : 12/07");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X, Audience:A)");
        telemetry.update();

        timer.reset();
        while (timer.time() < 10) {
            if (hzGamepad.getButtonBPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonAPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.AUDIENCE;
                telemetry.addData("Playing Alliance Selected : ", "AUDIENCE");
                break;
            }
            telemetry.addData("10s time : Default Alliance A : %.3f", timer.time());
            telemetry.update();
        }

        telemetry.update();
        sleep(500);

        //***** Select Start Pose ******
        timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (timer.time() < 10) {
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.AUDIENCE){
                telemetry.addData("Default Start Pose : ", "ORIGIN_FIELD");
                startPose = GameField.ORIGIN_FIELD;
                break;
            }
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = GameField.RED_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonAPress()) {
                    startPose = GameField.RED_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = GameField.BLUE_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonAPress()) {
                    startPose = GameField.BLUE_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            telemetry.addData("Start Pose : ", "ORIGIN_FIELD");
            telemetry.addData("10s Timer : Default Pose : ORIGIN_FIELD : %.3f", timer.time());
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

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("PoseStorage.poseSetInAutonomous : ",PoseStorage.poseSetInAutonomous);
        telemetry.addData("PoseStorage.currentPose : ",PoseStorage.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("hzDrive.driveMode : ", hzDrive.driveMode);
        telemetry.addData("hzDrive.poseEstimate :", hzDrive.poseEstimate);

        // Print pose to telemetry
        telemetry.addData("hzVuforia1.visibleTargetName : ", hzVuforia.visibleTargetName);
        telemetry.addData("hzVuforia1.poseVuforia :", hzVuforia.poseVuforia);
        //telemetry.addData("hzVuforia.vuforiaFirstAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaFirstAngle));
        //telemetry.addData("hzVuforia.vuforiaSecondAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaSecondAngle));
        //telemetry.addData("hzVuforia.vuforiaThirdAngle: %.3f°",Math.toDegrees(hzVuforia.vuforiaThirdAngle));

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
        telemetry.addData("hzMagazine.moveMagazineToLaunchState",hzMagazine.moveMagazineToLaunchState);
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("hzMagazine.moveMagazineToCollectState",hzMagazine.moveMagazineToCollectState);
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazine.magazineCollectTouchSensor.isPressed());


        //********** Intake Debug *******
        //telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepad.getDpad_downPress());
        //telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepad.getDpad_upPress());
        //telemetry.addData("intakeMotor.isBusy()", hzIntake.intakeMotor.isBusy());
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
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchController.lclaunchMotorPower);
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
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

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

        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());

        //telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());
        //telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

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
