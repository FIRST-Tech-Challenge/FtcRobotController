package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.HzGamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzSubsystem1;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Hazmat TeleOp", group = "00-Teleop")
public class HzTeleOp extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadController hzGamepadController;
    public HzDrive hzDrive;
    public HzSubsystem1 hzSubsystem1;
    //TODO: Replace name of Subsystem1 and Declare more subsystems

    //public HzVuforia hzVuforia1;
    public Pose2d startPose = HzGameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        hzDrive = new HzDrive(hardwareMap);
        hzSubsystem1 = new HzSubsystem1(hardwareMap);
        //TODO: Replace name of Subsystem1 and Declare more subsystems

        /* Create Controllers */
        hzGamepadController = new HzGamepadController(gamepad1, gamepad2, hzDrive, hzSubsystem1);

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( HzGameField.poseSetInAutonomous == true) {
            hzDrive.getLocalizer().setPoseEstimate(HzGameField.currentPose);
        } else {
            hzDrive.getLocalizer().setPoseEstimate(startPose);
        }

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        //TODO: Add code for any initial state
        //hzSubsystem1.setIntakeReleaseOpen();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                hzGamepadController.runByGamepadControl();

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        HzGameField.poseSetInAutonomous = false;
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
