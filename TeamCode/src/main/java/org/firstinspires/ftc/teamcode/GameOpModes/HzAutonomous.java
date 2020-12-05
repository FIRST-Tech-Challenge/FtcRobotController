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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    public GameField.PLAYING_ALLIANCE playingAlliance = GameField.PLAYING_ALLIANCE.AUDIENCE;
    public Pose2d startPose = hzGameField.ORIGIN_FIELD;

    //int playingAlliance = 1; //1 for Red, -1 for Blue
    //int startLine = 1 ; //0 for inner, 1 for outer
    boolean parked = false ;

    public enum TARGET_ZONE{
        A,
        B,
        C,
        UNKNOWN;
    };

    public TARGET_ZONE targetZone = TARGET_ZONE.A;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap, hzGameField);
        hzMagazine = new Magazine(hardwareMap);
        hzIntake = new Intake(hardwareMap);

        hzLauncher = new Launcher(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzLaunchController = new LaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, playingAlliance, hzDrive);
        hzGamepad1 = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        hzVuforia1 = new HzVuforia(hardwareMap);

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

        hzVuforia1.activateVuforiaTensorFlow();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Run Vuforia Tensor Flow
            targetZone = hzVuforia1.runVuforiaTensorFlow();


            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive() && !parked) {

                hzVuforia1.deactivateVuforiaTensorFlow();

                // Example spline path from SplineTest.java
                // Make sure the start pose matches with the localizer's start pose
                Trajectory traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .splineTo(new Vector2d(45, 45), 0)
                        .build();

                hzDrive.followTrajectory(traj);

                sleep(2000);

                hzDrive.followTrajectory(
                        hzDrive.trajectoryBuilder(traj.end(), true)
                                .splineTo(new Vector2d(15, 15), Math.toRadians(180))
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

        hzVuforia1.deactivateVuforiaTensorFlow();
    }

    public void initialConfiguration(){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        telemetry.addData("Compile time : ", "4:58 : 11/24");

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

        // Print pose to telemetry
        telemetry.addData("PoseEstimate : x", hzDrive.poseEstimate.getX());
        telemetry.addData("PoseEstimate : y", hzDrive.poseEstimate.getY());
        telemetry.addData("PoseEstimate : heading", Math.toDegrees(hzDrive.poseEstimate.getHeading()));

        telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        telemetry.addData("TARGET_ZONE Detected", targetZone);
        // Print pose to telemetry
        telemetry.addData("PoseVuforia : x", hzVuforia1.poseVuforia.getX());
        telemetry.addData("PoseVuforia : y", hzVuforia1.poseVuforia.getY());
        telemetry.addData("PoseVuforia : heading", Math.toDegrees(hzVuforia1.poseVuforia.getHeading()));

        telemetry.update();

    }
}
