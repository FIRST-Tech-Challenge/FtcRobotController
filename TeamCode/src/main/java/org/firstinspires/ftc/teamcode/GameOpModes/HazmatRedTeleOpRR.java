package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "Hazmat Red TeleOp RR FieldCentric", group = "00-Teleop")
public class HazmatRedTeleOpRR extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad1;
    public SampleMecanumDrive hzDrive;
    public HzVuforia hzVuforia1;
    public Pose2d startPose;
    int playingAlliance = 1; //1 for Red, -1 for Blue
    //TODO : Create another TeleOp for Red

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new SampleMecanumDrive(hardwareMap);
        hzGamepad1 = new HzGamepad(gamepad1);
        hzVuforia1 = new HzVuforia(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        hzDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO : When implementing Autonomous Mode uncomment this section.
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);

        // TODO : When in game comment below, so that Pose is retrieved from PoseStorage
        startPose = (new Pose2d(-68,24,Math.toRadians(0))); // Blue Inner Start Line
        //startPose = (new Pose2d(-68,48,Math.toRadians(0))); // Blue Outer Start Line
        //startPose = (new Pose2d(-68,-24,Math.toRadians(0))); // Red Inner Start Line
        //startPose = (new Pose2d(-68,-48,Math.toRadians(0))); // Red Outer Start Line
        hzDrive.setPoseEstimate(startPose);

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Activate Vuforia Navigation
            hzVuforia1.activateVuforiaNavigation();

            //Run Vuforia Navigation
            hzVuforia1.runVuforiaNavigation();

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {

                hzVuforia1.runVuforiaNavigation();

                //Run Robot based on field centric gamepad input, aligned to playing alliance direction
                hzGamepad1.runByGamepadRRDriveModes(this, hzDrive, playingAlliance);

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        hzVuforia1.deactivateVuforiaNavigation();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        // Print pose to telemetry
        telemetry.addData("Drive Mode : ", hzGamepad1.driveMode);
        telemetry.addData("PoseEstimate : x", hzGamepad1.poseEstimate.getX());
        telemetry.addData("PoseEstimate : y", hzGamepad1.poseEstimate.getY());
        telemetry.addData("PoseEstimate : heading", Math.toDegrees(hzGamepad1.poseEstimate.getHeading()));

        telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        // Print pose to telemetry
        telemetry.addData("PoseVuforia : x", hzVuforia1.poseVuforia.getX());
        telemetry.addData("PoseVuforia : y", hzVuforia1.poseVuforia.getY());
        telemetry.addData("PoseVuforia : heading", Math.toDegrees(hzVuforia1.poseVuforia.getHeading()));

        telemetry.update();

    }
}
