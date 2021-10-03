package org.firstinspires.ftc.teamcode.TestingOpModes.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.TestingOpModes.GamepadTestController;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Subsystem1 Template", group = "Test")
@Disabled
public class Test_Subsystem1_Template extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public SubsystemTemplate subsystemTemplate;

    //public HzVuforia hzVuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        subsystemTemplate = new SubsystemTemplate(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        subsystemTemplate.initSubsystem1();

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
                gamepadTestController.runByGamepadControl();

                //Add Test Code here
                if (gamepadTestController.getDpad_downPress()) {
                    if(subsystemTemplate.getSubsystemMotorState() == SubsystemTemplate.SUBSYSTEM1_MOTOR_STATE.STATE1) {
                        subsystemTemplate.startForwardSubsystem1Motor();
                    } else if(subsystemTemplate.getSubsystemMotorState() == SubsystemTemplate.SUBSYSTEM1_MOTOR_STATE.STATE2) {
                        subsystemTemplate.stopSubsystem1Motor();
                    }
                }

                //Reverse Intake motors and run - in case of stuck state)
                if (gamepadTestController.getDpad_upPersistent()) {
                    subsystemTemplate.startReverseSubsystem1Motor();
                } else if (subsystemTemplate.getSubsystemMotorState() == SubsystemTemplate.SUBSYSTEM1_MOTOR_STATE.STATE3){
                    subsystemTemplate.stopSubsystem1Motor();
                }

                if(HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("Subsystem1 State : ", subsystemTemplate.getSubsystemMotorState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
