package org.firstinspires.ftc.teamcode.mechaninisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotVars;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadController;

public class RobotStateHandler {
    private HardwareMap hardwareMap;
    private final RobotVars robotVars;

    public static Drivetrain drive;
    public static Intake intake;
    public static Outtake outtake;
    public static Stopper stopper;
    public static Transfer transfer;
    public static Limelight limelight;

    public RobotStateHandler(HardwareMap hardwareMapRef, RobotVars robotVarsRef) {
        hardwareMap = hardwareMapRef;
        robotVars = robotVarsRef;

        drive = new Drivetrain(hardwareMap, robotVars);
        intake = new Intake(hardwareMap, robotVars);
        outtake = new Outtake(hardwareMap, robotVars);
        stopper = new Stopper(hardwareMap, robotVars, outtake);
        transfer = new Transfer(hardwareMap, robotVars);
        limelight = new Limelight(hardwareMap, robotVars);

    }



    public void Init() {
        limelight.startScript();
    }

    public void RunMenu(GamepadController gamepadController) {

        if (gamepadController.setTeamToBlue()) {
            robotVars.allianceColor = AllianceColor.Blue;
            robotVars.setInitPoses();

            drive.follower.setPose(robotVars.startPose);
        } else if (gamepadController.setTeamToRed()) {
            robotVars.allianceColor = AllianceColor.Red;
            robotVars.setInitPoses();

            drive.follower.setPose(robotVars.startPose);
        }

        if (gamepadController.setStartPoseToWall()) {
            robotVars.startsInCorner = false;
            robotVars.setInitPoses();

            drive.follower.setPose(robotVars.startPose);
        } else if (gamepadController.setStartPoseToCorner()) {
            robotVars.startsInCorner = true;
            robotVars.setInitPoses();

            drive.follower.setPose(robotVars.startPose);
        }

    }


    public static void RunStateMachine(GamepadController gamepad) {
        intake.runIntake_ToggleMode(gamepad.toggleIntakeButton(), gamepad.toggleReverseIntakeButton());
        outtake.runOuttake_ToggleMode(gamepad.toggleOuttakeButton(), gamepad.toggleReverseOuttakeButton());
        stopper.runStopper_ToggleMode(gamepad.shootButton());
        transfer.runTransfer_ToggleMode(gamepad.toggleTransferButton(), gamepad.toggleReverseIntakeButton());

    }
}
