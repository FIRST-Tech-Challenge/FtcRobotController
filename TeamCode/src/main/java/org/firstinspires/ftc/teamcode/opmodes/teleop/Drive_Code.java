package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.mechaninisms.RobotStateHandler.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotVars;
import org.firstinspires.ftc.teamcode.mechaninisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadController;

@TeleOp(name = "Main TeleOP", group = "Linear OpMode")
public class Drive_Code extends LinearOpMode {
    public Servo stopperServo;

//    public static double outtakeSpeed = 1500;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotVars robotVars = new RobotVars();
        RobotStateHandler robotStateHandler = new RobotStateHandler(hardwareMap, robotVars);
        GamepadController gamepadController = new GamepadController(gamepad1, gamepad2);
        stopperServo = hardwareMap.get(Servo.class, "stopper");

        RobotVars.init_TeleOp(telemetry);

        // Test line:
//        robotVars.constants.outtakeMotorSpeed = outtakeSpeed;



        while (!opModeIsActive()) {

            robotStateHandler.RunMenu(gamepadController);

            telemetry.addData("TeleOp: ", (robotVars.allianceColor == AllianceColor.Red ? "Red " : "Blue ") + (robotVars.startsInCorner ? "Corner" : "Wall"));

            telemetry.update();

        }

        //-----------------------------------------------------------------
        // Initialization ↑ and Start ↓
        //-----------------------------------------------------------------


        waitForStart();

        robotStateHandler.Init();;

        //-----------------------------------------------------------------
        // Start ↑ and Primary Loop ↓
        //-----------------------------------------------------------------

        // Primary loop
        while (opModeIsActive() && !isStopRequested()) {


            RobotStateHandler.drive.runDriveCode(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepadController.hiveCentricDriveButton()
            );

            RobotStateHandler.RunStateMachine(gamepadController);

//            if (gamepad2.dpadDownWasPressed()) {
//                robotVars.constants.outtakeMotorSpeed = robotVars.constants.outtakeMotorSpeed - 10;
//            } else if (gamepad2.dpadUpWasPressed()) {
//                robotVars.constants.outtakeMotorSpeed = robotVars.constants.outtakeMotorSpeed + 10;
//            }

            telemetry.addData("VELOCITY", outtake.outtakeMotor.getVelocity());
            telemetry.addData("TeleOp: ", (robotVars.allianceColor == AllianceColor.Red ? "Red " : "Blue ") + (robotVars.startsInCorner ? "Corner" : "Wall"));
//            telemetry.addData("Start Pose: ", robotVars.startPose);
            telemetry.addData("Left Front", RobotStateHandler.drive.leftFront.getCurrentPosition());
            telemetry.addData("Left Rear: ", RobotStateHandler.drive.leftBack.getCurrentPosition());
            telemetry.addData("Right Front", RobotStateHandler.drive.rightFront.getCurrentPosition());
            telemetry.addData("Right Rear: ", RobotStateHandler.drive.rightBack.getCurrentPosition());
//            telemetry.addData("Follower: ", RobotStateHandler.drive.follower.getPose().getX() + " " + RobotStateHandler.drive.follower.getPose().getY() + ", " + Math.toDegrees(RobotStateHandler.drive.follower.getPose().getHeading()));
            telemetry.addData("is Outtake Active: ", robotVars.isOuttakeActive);
            telemetry.addData("is Intake & Transfer Active: ", robotVars.isIntakeActive + " " + robotVars.isTransferActive);
//            telemetry.addData("is Stopper Down: ", robotVars.isStopperDown);
//            telemetry.addData("stopperDownPos: ", robotVars.constants.stopperDownPosition);
            telemetry.addData("outtake Speed: ", robotVars.constants.outtakeMotorSpeed);




            telemetry.update();

        }


        // Emergency shut down
        RobotStateHandler.drive.setPowers(0, 0, 0, 0);


    }
}
