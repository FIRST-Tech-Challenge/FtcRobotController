package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@TeleOp(name = "Hazmat TeleOp RR FieldCentric", group = "00-Teleop")
public class HzTeleOpRRFieldCentric extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    public GameField hzGameField;
    //public SampleMecanumDrive hzDrive;
    public HzDrive hzDrive;
    public Magazine hzMagazine;
    public Intake hzIntake;
    public LaunchController hzLaunchController;
    public Launcher hzLauncher;
    public Arm hzArm;

    public HzVuforia hzVuforia1;
    public Pose2d startPose = hzGameField.ORIGIN_FIELD;
    //int playingAlliance = 0; //1 for Red, -1 for Blue, 0 for Audience
    //TODO : Create another TeleOp for Red

    public GameField.PLAYING_ALLIANCE playingAlliance = GameField.PLAYING_ALLIANCE.AUDIENCE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap, hzGameField);
        hzMagazine = new Magazine(hardwareMap);
        hzIntake = new Intake(hardwareMap);

        hzLauncher = new Launcher(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzLaunchController = new LaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, playingAlliance, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        initialConfiguration();

        //hzVuforia1 = new HzVuforia(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        hzDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO : When implementing Autonomous Mode uncomment this section.
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //hzDrive.setPoseEstimate(PoseStorage.currentPose);
        // TODO : When in game comment below, so that Pose is retrieved from PoseStorage
        //startPose = GameField.BLUE_INNER_START_LINE;
        if ( PoseStorage.poseSetInAutonomous == true) {
            hzDrive.setPoseEstimate(PoseStorage.currentPose);
        } else {
            hzDrive.setPoseEstimate(startPose);
        }

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

                //Run Robot based on field centric gamepad input, aligned to playing alliance direction
                //hzGamepad1.runByGamepadRRDriveModes(hzDrive, playingAlliance*/);
                hzGamepad.runByGamepad();
                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        hzVuforia1.deactivateVuforiaNavigation();
        PoseStorage.poseSetInAutonomous = false;
    }

    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X, Audience:A)");
        telemetry.update();

        timer.reset();
        while (timer.time() < 10) {
            if (hzGamepad.getButtonBPress()) {
                playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonAPress()) {
                playingAlliance = GameField.PLAYING_ALLIANCE.AUDIENCE;
                telemetry.addData("Playing Alliance Selected : ", "AUDIENCE");
                break;
            }
            telemetry.addData("10s time : Default Alliance A : %.3f", timer.time());
            telemetry.update();
        }
        hzGamepad.playingAlliance = playingAlliance;
        telemetry.update();
        sleep(2000);

        //***** Select Start Pose ******
        timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (timer.time() < 10) {
            if (playingAlliance == GameField.PLAYING_ALLIANCE.AUDIENCE){
                telemetry.addData("Default Start Pose : ", "ORIGIN_FIELD");
                startPose = GameField.ORIGIN_FIELD;
                break;
            }
            if (playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
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
            if (playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
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
        sleep(2000);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("hzGamepad1.playingAlliance : ", hzGamepad.playingAlliance);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate : x", hzDrive.poseEstimate.getX());
        telemetry.addData("PoseEstimate : y", hzDrive.poseEstimate.getY());
        telemetry.addData("PoseEstimate : heading", Math.toDegrees(hzDrive.poseEstimate.getHeading()));

        //telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia : x", hzVuforia1.poseVuforia.getX());
        //telemetry.addData("PoseVuforia : y", hzVuforia1.poseVuforia.getY());
        //telemetry.addData("PoseVuforia : heading", Math.toDegrees(hzVuforia1.poseVuforia.getHeading()));

        //******* Magazine Debug ********
        //telemetry.addData("getDistance(DistanceUnit.CM)",hzMagazine.magazine_distance);
        /*switch (hzMagazine.getMagazineRingCount()){
            case ZERO:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_0");
                break;
            }
            case ONE:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_1");
                break;
            }
            case TWO:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_2");
                break;
            }
            case THREE:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_3");
                break;
            }
        }*/
        switch (hzMagazine.getMagazinePosition()) {
            case AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_COLLECT");
                break;
            }
            case AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_ERROR");
                break;
            }
        }
        telemetry.addData("magazineLaunchTouchSensor.getState()", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState()", hzMagazine.magazineCollectTouchSensor.isPressed());

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
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchController.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchController.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchController.lcTarget);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchController.distanceFromTarget);
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchController.lclaunchMotorPower);
        telemetry.addData("hzDrive.drivePointToAlign : ", hzDrive.drivePointToAlign);


        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncher.launcherRingPlungerServo.getPosition());
        //telemetry.addData("hzLauncher.launcherMotorPower : ", hzLauncher.launcherMotorPower);

        switch (hzLauncher.getLauncherState()){
            case RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
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
