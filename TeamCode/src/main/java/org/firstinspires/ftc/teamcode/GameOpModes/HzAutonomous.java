package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "Hazmat Red Autonomous Outer", group = "00-Autonomous")
public class HzAutonomous extends LinearOpMode {

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
    public Pose2d startPose = GameField.BLUE_INNER_START_LINE;

    //int playingAlliance = 1; //1 for Red, -1 for Blue
    //int startLine = 1 ; //0 for inner, 1 for outer
    boolean parked = false ;

    public GameField.TARGET_ZONE targetZone = GameField.TARGET_ZONE.A;
    Vector2d targetZoneVector = GameField.TARGET_ZONE_A;

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

        hzVuforia = new HzVuforia(hardwareMap);



        initialConfiguration();

        /*
        // Set initial pose
        if (playingAlliance == 1)  { //Red
            if (startLine == 0)   { //Inner Line
                drive.setPoseEstimate(new Pose2d(-68,-24,Math.toRadians(0))); // Red Inner Start Line
            } else  { //Outer Line
                drive.setPoseEstimate(new Pose2d(-68,-48,Math.toRadians(0))); // Red Outer Start Line
            }
        } else  { // Blue
            if (startLine == 0) { // Inner Line
                drive.setPoseEstimate(new Pose2d(-68,24,Math.toRadians(0))); // Blue Inner Start Line
            } else  { // Outer Line
                drive.setPoseEstimate(new Pose2d(-68,48,Math.toRadians(0))); // Blue Outer Start Line
            }
        }*/

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        hzVuforia.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Run Vuforia Tensor Flow
            targetZone = hzVuforia.runVuforiaTensorFlow();

            switch (targetZone) {
                case A :
                    targetZoneVector = GameField.TARGET_ZONE_A;
                    break;
                case B :
                    targetZoneVector = GameField.TARGET_ZONE_B;
                    break;
                case C :
                    targetZoneVector = GameField.TARGET_ZONE_C;
                    break;
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive() && !parked) {

                hzVuforia.deactivateVuforiaTensorFlow();

                // Example spline path from SplineTest.java
                // Make sure the start pose matches with the localizer's start pose
                Trajectory traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .splineTo(new Vector2d(0, 0), 0)
                        .build();

                hzDrive.followTrajectory(traj);

                sleep(2000);

                hzDrive.followTrajectory(
                        hzDrive.trajectoryBuilder(traj.end(), true)
                                .splineTo(targetZoneVector, Math.toRadians(-90))
                                .build()
                );

                //Move to Launching Position

                parked = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = hzDrive.getPoseEstimate();
        PoseStorage.poseSetInAutonomous = true;

        hzVuforia.deactivateVuforiaTensorFlow();
    }

    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X)");
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
            telemetry.addData("10s time : Default Alliance A : %.3f", timer.time());
            telemetry.update();
        }

        telemetry.update();
        sleep(500);

        //***** Select Start Pose ******
        timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (timer.time() < 10) {
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
            telemetry.addData("10s Timer : Default Pose : BLUE_INNER_START_LINE : %.3f", timer.time());
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
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);

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
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
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
