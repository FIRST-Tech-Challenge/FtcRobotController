package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.teamcode.hardware.camera;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class TeleOpMainBlue extends AutonBase {

    static final String TAG = "TeleOp Main Blue";
    static final int TAG_GROUP = 1;          // 1 = blue (tags 38-45), 2 = red (tags 30-37)
    static final double STICK_DEADBAND = 0.05;
    static final double AIM_TURN_SPEED = 1;
    static final double SCAN_ANGLE = 30;
    static final long SETTLE_MS = 150;

    private double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }

    // Scans at center, 30 right, then 30 left, then turns to face the center of the tags.

    private double scanAndAim(camera Camera) {
        Map<Integer, double[]> data = new HashMap<>();

        theHardwareMap.chImu.resetYaw();
        Camera.scanFrames(TAG_GROUP, 0, data);

        imuTurn(AIM_TURN_SPEED, SCAN_ANGLE);
        sleep(SETTLE_MS);
        Camera.scanFrames(TAG_GROUP, SCAN_ANGLE, data);

        imuTurn(AIM_TURN_SPEED, -2 * SCAN_ANGLE);
        sleep(SETTLE_MS);
        Camera.scanFrames(TAG_GROUP, -SCAN_ANGLE, data);

        double center = Camera.centerFromScans(data);

        if (Double.isNaN(center)) {
            imuTurn(AIM_TURN_SPEED, SCAN_ANGLE);
        } else {
            imuTurn(AIM_TURN_SPEED, center + SCAN_ANGLE);
        }
        return center;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int soundID = hardwareMap.appContext.getResources().getIdentifier(
                "oof", "raw", hardwareMap.appContext.getPackageName());

        if (soundID != 0) {
            SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        }

        initialize();

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        Outtake outtake = new Outtake(theHardwareMap, this);

        camera Camera = new camera(theHardwareMap, this);

        boolean slowMode = false;
        double lastAimAngle = Double.NaN;

        telemetry.addData("Robot", "Initialized successfully");
        telemetry.update();

        waitForStart();
        Camera.start();

        if (soundID != 0) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        }

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Main loop
        while (opModeIsActive()) {
            Camera.loop();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double drive = deadband(-1 * gamepad1.left_stick_y);
            double strafe = -1 * gamepad1.left_trigger + gamepad1.right_trigger;
            double twist = deadband(gamepad1.right_stick_x);

            // slow mode speed values
            double speed = slowMode ? 0.45 : 0.9;
            drive *= speed;
            strafe *= speed;
            twist *= speed;

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
            }

            if (currentGamepad1.b) {
                twist = Camera.Robotallignwithgoal(twist);
            }

            robotDrive.teleOpMechanum(drive, strafe, twist);

            /***************
             * Gamepad 2
             */
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                outtake.increasemotorspeed();
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                outtake.decreasemotorspeed();
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                lastAimAngle = scanAndAim(Camera);
            }

            /***************
             * Debug telemetry
             */
            telemetry.addData("Stick twist (raw)", gamepad1.right_stick_x);
            telemetry.addData("Final twist", twist);
            telemetry.addData("Drive / Strafe", drive + " / " + strafe);
            telemetry.addData("Slow mode", slowMode);
            telemetry.addData("Last Aim angle (deg)", Double.isNaN(lastAimAngle) ? "none / no tags" : String.format("%.2f", lastAimAngle));
            telemetry.update();
        }
    }
}