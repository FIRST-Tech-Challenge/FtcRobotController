package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.AutonBase;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.teamcode.hardware.camera;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class TeleOpMainRed extends AutonBase {

    static final String TAG = "TeleOp Main Red";
    static final int TAG_GROUP = 2;          // 1 = blue (tags 38-45), 2 = red (tags 30-37)
    static final double STICK_DEADBAND = 0.05; // stick values smaller than this are treated as 0
    static final double AIM_TURN_SPEED = 0.3;  // max turn power for the final imuTurn that faces the tags
    static final double SCAN_TURN_SPEED = 1.0; // power for the fast encoder turns between scans
    static final double SCAN_ANGLE = 30;       // degrees to look left and right of the starting heading
    static final long SETTLE_MS = 150;         // pause after each turn so the camera frame isn't blurry

    // zeroes out tiny stick values so a drifting stick doesn't move the robot
    private double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }

    // How far the robot is turned from its starting heading, read from the IMU (right is positive).
    // IMU yaw counts counterclockwise as positive, so it is flipped here.
    private double currentOffset() {
        return -theHardwareMap.chImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Scans at center, ~30 right, then ~30 left (fast encoder turns), then uses imuTurn to face
    // the center of the tags. The scan angles come from the real IMU heading, not the encoder estimate.
    // Returns the center angle from the starting heading (right is positive), or NaN if no tags were seen.
    private double scanAndAim(camera Camera) {
        Map<Integer, double[]> data = new HashMap<>();

        theHardwareMap.chImu.resetYaw();                          // current heading becomes 0
        Camera.scanFrames(TAG_GROUP, 0, data);                    // 1) center

        encoderTurn(SCAN_TURN_SPEED, SCAN_ANGLE);                 // turn right about 30
        sleep(SETTLE_MS);
        Camera.scanFrames(TAG_GROUP, currentOffset(), data);      // 2) right

        encoderTurn(SCAN_TURN_SPEED, -2 * SCAN_ANGLE);            // turn left about 60
        sleep(SETTLE_MS);
        Camera.scanFrames(TAG_GROUP, currentOffset(), data);      // 3) left

        double center = Camera.centerFromScans(data);
        double offset = currentOffset();                          // where the robot is pointing now
        theHardwareMap.chImu.resetYaw();                          // so imuTurn is relative to here

        if (Double.isNaN(center)) {
            imuTurn(AIM_TURN_SPEED, -offset);                     // no tags: go back to the starting heading
        } else {
            imuTurn(AIM_TURN_SPEED, center - offset);             // turn from here to the tag center
        }
        return center;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware (AutonBase.initialize creates theHardwareMap and sets up the IMU)
        initialize();

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        Outtake outtake = new Outtake(theHardwareMap, this);

        camera Camera = new camera(theHardwareMap, this);

        boolean slowMode = false;
        double lastAimAngle = Double.NaN; // result of the last Aim call (NaN = none yet / no tags seen)

        telemetry.addData("Robot", "Initialized successfully");
        telemetry.update();

        waitForStart();
        Camera.start();

        // gamepads used to detect button presses (current vs. previous loop pass)
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Main loop
        while (opModeIsActive()) {
            Camera.loop();

            // copy over the previous gamepads so we can compare what changed
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /***************
             * Gamepad 1 - driving
             */
            // mecanum drive - left stick y is negative because up is negative
            double drive = deadband(-1 * gamepad1.left_stick_y);
            double strafe = -1 * gamepad1.left_trigger + gamepad1.right_trigger;
            double twist = deadband(gamepad1.right_stick_x);

            // slow mode speed values
            double speed = slowMode ? 0.45 : 0.9;
            drive *= speed;
            strafe *= speed;
            twist *= speed;

            // left bumper = slow mode on, right bumper = slow mode off
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
            }

            // hold B = turn toward the goal using the camera
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

            // right bumper (pressed once) = scan center/right/left, then turn to face the center of the tags
            // NOTE: this blocks for a few seconds while it scans and turns
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