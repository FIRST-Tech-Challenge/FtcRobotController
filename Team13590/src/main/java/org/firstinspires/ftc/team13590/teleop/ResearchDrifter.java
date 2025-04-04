package org.firstinspires.ftc.team13590.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team13590.RobotHardware;
import org.firstinspires.ftc.team13590.VisionSoftware;

@Disabled
@TeleOp(name = "Drifter...", group = "Robot")
public class ResearchDrifter extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);


    private boolean coloredFound;
    private boolean yellowFound;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");
        limelight.pipelineSwitch(1);

        Pose3D botPose;

        // sounds
        int coloredSoundID = hardwareMap.appContext.getResources().getIdentifier("colored", "raw", hardwareMap.appContext.getPackageName());
        int yellowSoundID = hardwareMap.appContext.getResources().getIdentifier("yellow", "raw", hardwareMap.appContext.getPackageName());
        // preload sounds
        if (coloredSoundID != 0) {
            coloredFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, coloredSoundID);
        }
        if (yellowSoundID != 0) {
            yellowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, yellowSoundID);
        }
        telemetry.addData("gold resource", coloredFound ? "Found" : "NOT found\n Add colored.wav to /src/main/res/raw");
        telemetry.addData("silver resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw");
        SoundPlayer.getInstance().setMasterVolume(2);
        // initialization phase...
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        double posX = 0;
        double posY = 0;

        robot.init(false);
        //aptDetector.visionInit();
        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        robot.elbowDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        limelight.start();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (limelight.getLatestResult() != null) {
                 botPose = limelight.getLatestResult().getBotpose();
                posX = botPose.getPosition().x;
                posY = botPose.getPosition().y;
            }


            if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0) {
                drive = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1.1;
                turn = -gamepad1.right_stick_x;
            } else {
                drive = Math.round((drive * 0.7) / 0.01) * 0.01;
                strafe = Math.round((strafe * 0.7) / 0.01) * 0.01;
                turn = Math.round((turn * 0.7) / 0.01) * 0.01;
            }

            // BRAKE
            if (gamepad1.right_trigger != 0) { // slow down driving
                double multiplier = -gamepad1.right_trigger + 1; // reverse trigger (it goes from 0 to 1, bad!)
                drive = drive * multiplier;
                strafe = strafe * multiplier;
                turn = turn * multiplier;
            }

            // DETECT APT
                /*aptDetector.activeAPTscanner(-1);
                if (aptDetector.targetFound) {
                    telemetry.addData("APT found", "");
                    posX = aptDetector.detectedTag.robotPose.getPosition().x;
                    posY = aptDetector.detectedTag.robotPose.getPosition().y;
                }*/


            robot.driveRobotCentric(drive, strafe, turn);

            // telemetry
            telemetry.addData("Heading", robot.heading);
            telemetry.addData("Manual Driving", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("XY Robot Position:", "\n" + posX*39.3701 + ", " + posY*39.3701);
            telemetry.update();
            sleep(50);
        }
    }
}
