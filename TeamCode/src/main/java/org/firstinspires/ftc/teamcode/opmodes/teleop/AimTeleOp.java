package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AimConfig;
import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.AimController;

/*
 * Main TeleOp plus auto-aim.
 * Left stick: drive/strafe. Right stick: turn.
 * Left trigger: feed (hold). Right bumper: toggle intake.
 * Left bumper (hold): run the flywheel backward to clear a stuck ball.
 * Right trigger (hold): auto-aim at the closest tag cluster. Once lined up, intake and feeder start.
 *   With no tag in view the robot stops (the sticks are ignored while the trigger is held).
 */
@TeleOp(name = "Aim TeleOp", group = "Competition")
public class AimTeleOp extends OpMode {

    private Robot robot;
    private final AimController aim = new AimController();

    @Override
    public void init() {
        RobotLog.i("AimTeleOp: Initializing robot");
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void loop() {
        robot.vision.setServoPosition(AimConfig.LIMELIGHT_SERVO_POS); // every loop so Dashboard changes apply

        boolean aiming = gamepad1.right_trigger >= 0.3;
        boolean feedRequested = gamepad1.left_trigger >= 0.3;

        if (aiming && robot.vision.hasAimTarget()) {
            aim.calculate(robot.vision.getAimTx(), robot.vision.getAimTy(),
                    robot.odo.getHeadingVelocityRadPerSec());
            robot.drive.drive(aim.forward(), gamepad1.left_stick_x, aim.turn());
            if (aim.isAimed() && gamepad1.right_trigger >= 0.5) {
                robot.intake.on();
                feedRequested = true;
            }
        } else if (aiming) {
            robot.drive.drive(0, 0, 0); // no target: hold still
        } else {
            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.left_bumper) {
            robot.launcher.reverse(LauncherConfig.REVERSE_POWER);
        } else {
            robot.launcher.spinUp();
            if (feedRequested) {
                robot.launcher.feed();
            } else {
                robot.launcher.retractFeeder();
            }
        }

        if (gamepad1.rightBumperWasPressed()) {
            robot.intake.toggle();
        }

        robot.update();
    }

    @Override
    public void stop() {
        RobotLog.i("AimTeleOp: Stopping robot");
        robot.stop();
    }
}
