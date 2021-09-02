package org.firstinspires.ftc.teamcode.TestingOpModes.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.HzGameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzSubsystem1;
import org.firstinspires.ftc.teamcode.TestingOpModes.HzGamepadTestController;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Subsystem1 Template", group = "Test")
public class Test_Subsystem1_Template extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadTestController hzGamepadTestController;
    public HzDrive hzDrive;
    public HzSubsystem1 hzSubsystem1;

    //public HzVuforia hzVuforia1;
    public Pose2d startPose = HzGameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        hzDrive = new HzDrive(hardwareMap);
        hzSubsystem1 = new HzSubsystem1(hardwareMap);
        /* Create Controllers */
        hzGamepadTestController = new HzGamepadTestController(gamepad1,hzDrive);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        hzSubsystem1.initSubsystem1();

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
                hzGamepadTestController.runByGamepadControl();

                //Add Test Code here
                if (hzGamepadTestController.getDpad_downPress()) {
                    if(hzSubsystem1.getSubsystemMotorState() == HzSubsystem1.SUBSYSTEM1_MOTOR_STATE.STATE1) {
                        hzSubsystem1.startForwardSubsystem1Motor();
                    } else if(hzSubsystem1.getSubsystemMotorState() == HzSubsystem1.SUBSYSTEM1_MOTOR_STATE.STATE2) {
                        hzSubsystem1.stopSubsystem1Motor();
                    }
                }

                //Reverse Intake motors and run - in case of stuck state)
                if (hzGamepadTestController.getDpad_upPersistent()) {
                    hzSubsystem1.startReverseSubsystem1Motor();
                } else if (hzSubsystem1.getSubsystemMotorState() == HzSubsystem1.SUBSYSTEM1_MOTOR_STATE.STATE3){
                    hzSubsystem1.stopSubsystem1Motor();
                }

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
        telemetry.addData("Battery Power : ", hzDrive.getBatteryVoltage(hardwareMap));

        telemetry.addData("Subsystem1 State : ", hzSubsystem1.getSubsystemMotorState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
