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


/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "Hazmat TeleOp RR FieldCentric", group = "00-Teleop")
public class HazmatTeleOpRRFieldCentric extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad1;
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
        hzLaunchController = new LaunchController(hardwareMap, hzLauncher, hzMagazine, playingAlliance, hzDrive);
        hzGamepad1 = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        initialConfiguration();

        //hzVuforia1 = new HzVuforia(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        hzDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO : When implementing Autonomous Mode uncomment this section.
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);
        // TODO : When in game comment below, so that Pose is retrieved from PoseStorage
        hzDrive.setPoseEstimate(startPose);

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
                hzGamepad1.runByGamepad();
                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        hzVuforia1.deactivateVuforiaNavigation();
    }

    public void initialConfiguration(){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        telemetry.addData("Compile time : ", "4:41 : 11/24");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X, Audience:A)");
        telemetry.update();

        timer.reset();
        while (timer.time() < 10) {
            if (hzGamepad1.getButtonBPress()) {
                hzGamepad1.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;}
            if (hzGamepad1.getButtonXPress()) {
                hzGamepad1.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;}
            if (hzGamepad1.getButtonAPress()) {
                hzGamepad1.playingAlliance = GameField.PLAYING_ALLIANCE.AUDIENCE;
                telemetry.addData("Playing Alliance Selected : ", "AUDIENCE");
                break;}
            telemetry.addData("10s Time out : Default Alliance selected : Audience A : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();

        //***** Select Start Pose ******
        timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (timer.time() < 10) {
            if (playingAlliance == GameField.PLAYING_ALLIANCE.AUDIENCE){
                telemetry.addData("Start Pose : ", "ORIGIN_FIELD");
                startPose = GameField.ORIGIN_FIELD;
                break;
            }
            if (playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad1.getButtonAPress()) {
                    startPose = GameField.RED_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad1.getButtonAPress()) {
                    startPose = GameField.RED_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad1.getButtonAPress()) {
                    startPose = GameField.BLUE_INNER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad1.getButtonAPress()) {
                    startPose = GameField.BLUE_OUTER_START_LINE;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            telemetry.addData("Start Pose : ", "ORIGIN_FIELD");
            telemetry.addData("10s Time out : Default Pose selected : ORIGIN_FIELD : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);


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
        telemetry.addData("getDistance(DistanceUnit.CM)",hzMagazine.magazine_distance);
        switch (hzMagazine.getMagazineRingCount()){
            case MAGAZINE_RINGS_0:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_0");
                break;
            }
            case MAGAZINE_RINGS_1:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_1");
                break;
            }
            case MAGAZINE_RINGS_2:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_2");
                break;
            }
            case MAGAZINE_RINGS_3:  {
                telemetry.addData("hzMagazine.getMagazineRingCount()", "MAGAZINE_RINGS_3");
                break;
            }
        }
        switch (hzMagazine.getMagazinePosition()) {
            case MAGAZINE_AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case MAGAZINE_AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_COLLECT");
                break;
            }
            case MAGAZINE_AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition()", "MAGAZINE_AT_ERROR");
                break;
            }
        }
        telemetry.addData("magazineLaunchTouchSensor.getState()", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState()", hzMagazine.magazineCollectTouchSensor.isPressed());

        //***** Arm Debug ****
        telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        telemetry.update();

    }
}
