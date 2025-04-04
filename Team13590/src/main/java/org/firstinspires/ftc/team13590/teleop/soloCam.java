package org.firstinspires.ftc.team13590.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team13590.FieldLocalization;
import org.firstinspires.ftc.team13590.RobotHardware;

@Disabled
@TeleOp(name = "SoloCameraOp", group = "Solor")
public class soloCam extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    FieldLocalization localization = new FieldLocalization(12, robot);


    private boolean coloredFound;
    private boolean yellowFound;
    private int NEWelbowPos;
    private int stickCounter = 0;
    private int yawCounter = 0;
    private boolean turnYaw = false;
    private  boolean calibratePerpendicular = false;
    private Pose3D botPose = null;

    // uses A, B, X, Y, & RSB + LSB
    private void clawDrive(){
        if (gamepad1.x) { // close claw
            robot.setClawPosition(robot.enable, robot.pass, robot.pass);
        } else if (gamepad1.b) { // open claw
            robot.setClawPosition(robot.disable, robot.pass, robot.pass);
        }

        if (gamepad2.a) { // down claw
            robot.clawAxial.setPosition(robot.CLAW_DOWN);
            calibratePerpendicular = false;
        } else if (gamepad2.y) { // claw mid
            robot.clawAxial.setPosition(robot.CLAW_MID);
            calibratePerpendicular = false;
        }

        // drive yaw (toggle)
        if (gamepad1.right_stick_button && yawCounter > 3){
            yawCounter = 0;
            turnYaw = !turnYaw;
            if (turnYaw) {
                robot.clawYaw.setPosition(robot.YAW_RIGHT);
            } else { robot.clawYaw.setPosition(robot.YAW_MID); }
        }

        // toggle claw mode
        if (gamepad1.left_stick_button && stickCounter > 3) { // set claw perpendicular to floor
            calibratePerpendicular = !calibratePerpendicular;
            stickCounter = 0;
        }
    }

    // sticks
    private void driveSticks(){
        robot.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
    }

    // uses all of dpad
    private void dpadChecker(){
        if (gamepad2.dpad_up) {
            NEWelbowPos = ((int) (robot.ELBOW_PARALLEL + robot.angleConvert(15)));
        } else if (gamepad2.dpad_right) {
            NEWelbowPos = (int) (robot.angleConvert(33));
        } else if (gamepad2.dpad_left) {
            NEWelbowPos = ((int) robot.ELBOW_BACKWARD_PARALLEL);
        } else if (gamepad2.dpad_down) { // slightly off ground
            NEWelbowPos = (int) (robot.ELBOW_COLLAPSED);
        }
    }

    // LB + RB for extender
    private int bumperSolver(){

        int Lval = gamepad1.left_bumper ? -1 : 0;
        int Rval = gamepad1.right_bumper ?  1 : 0;

        return Lval + Rval;
    }

    // RT & LT for elbow
    private void driveElbow() {
        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            double elbowFactor;
            if (gamepad1.left_trigger > 0.5) {
                elbowFactor = Math.pow(3 * gamepad1.left_trigger, 2) * -robot.ELBOW_FUDGE_FACTOR;
            } else if (gamepad1.right_trigger > 0.5) {
                elbowFactor = Math.pow(3 * gamepad1.right_trigger, 2) * robot.ELBOW_FUDGE_FACTOR;
            } else {
                elbowFactor = robot.ELBOW_FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
            }

            // avoid going past 0
            if (robot.elbowDrive.getCurrentPosition() + (int) elbowFactor < 0) {
                elbowFactor -= (robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);
            }
            // if they are drive and change the target position
            NEWelbowPos = robot.elbowDrive.getCurrentPosition() + (int) elbowFactor;
        } else {
            if (botPose != null) {
                NEWelbowPos = localization.elbowAssistantPassive(botPose, robot.heading);
            }
        }
    }


    @Override
    public void runOpMode() {
        // timers
        int timesCounted = 0;
        // sounds
        int coloredSoundID = hardwareMap.appContext.getResources().getIdentifier("colored", "raw", hardwareMap.appContext.getPackageName());
        int yellowSoundID   = hardwareMap.appContext.getResources().getIdentifier("yellow",   "raw", hardwareMap.appContext.getPackageName());
        // preload sounds
        if (coloredSoundID != 0){ coloredFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, coloredSoundID); }
        if (yellowSoundID != 0){ yellowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, yellowSoundID); }
        telemetry.addData("gold resource",   coloredFound ?   "Found" : "NOT found\n Add colored.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw" );
        SoundPlayer.getInstance().setMasterVolume(2);
        // initialization phase...

        boolean calibrateParallel = false;

        double prevElbowPos;

        robot.init(false);

        // extension encoder setup
        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // set limelight pos if u find one
            if (robot.limelight.getLatestResult() != null) {
                botPose = robot.limelight.getLatestResult().getBotpose();
            } else {
                telemetry.addData("not detecting", "");
            }
            prevElbowPos = robot.elbowDrive.getCurrentPosition();

            robot.driveExtenderPosition(bumperSolver());

            driveElbow();
            driveSticks();
            dpadChecker();
            clawDrive();

            robot.elbowDrive.setTargetPosition(NEWelbowPos);

            // telemetry
            telemetry.addData("Heading", robot.heading);
            telemetry.addData("Elbow Position", robot.elbowDrive.getCurrentPosition() / robot.ARM_COUNTS_PER_DEGREE);
            telemetry.addData("Extender Position", robot.extensionDrive.getCurrentPosition() / robot.EXTENSION_COUNTS_PER_REV);
            telemetry.addData("Claw Calibration", "Parallel, Perpendicular", calibrateParallel, calibratePerpendicular);
            telemetry.update();

            // check for elbow stalls
            if (prevElbowPos == robot.elbowDrive.getCurrentPosition() && robot.elbowDrive.getPower() >= 1.0){
                if (Math.abs(robot.elbowDrive.getCurrentPosition() - robot.elbowDrive.getTargetPosition()) > 4) {
                    timesCounted++;
                    if (timesCounted >= 10) {
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, coloredSoundID);
                    }
                }
            }

            yawCounter++;
            stickCounter++;
            sleep(50);
        }
    }
}
