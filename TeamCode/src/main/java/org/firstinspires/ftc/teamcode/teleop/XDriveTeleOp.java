package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.XDrive;

/**
 * Driver-controlled OpMode for an X-drive robot.
 *
 * Gamepad 1:
 *   left stick          drive and strafe
 *   right stick (x)     turn
 *   left bumper (hold)  precision mode - everything moves slowly for lining up
 *   right bumper (hold) robot-centric override, like driving an RC car
 *   Y                   toggle field-centric drive on/off
 *   A                   set the current heading as "forward" for field-centric drive
 */
@TeleOp(name = "X-Drive TeleOp (drive only)", group = "Drive")
@Disabled  // Superseded by CompTeleOp. Kept as a plain-SDK reference - delete @Disabled to use it.
public class XDriveTeleOp extends LinearOpMode {

    /**
     * How the Control Hub is mounted on the robot. Change these two to match your build, or
     * field-centric drive will read the heading off the wrong axis.
     */
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /** Stick movement smaller than this is treated as zero, so a worn stick doesn't creep. */
    private static final double DEADBAND = 0.05;

    private static final double NORMAL_SPEED    = 1.0;
    private static final double PRECISION_SPEED = 0.35;

    /** Turning is scaled down separately - full-power turns are hard to control. */
    private static final double TURN_SCALE = 0.8;

    private boolean fieldCentric = true;

    @Override
    public void runOpMode() {
        XDrive drive = new XDrive(hardwareMap, LOGO_DIRECTION, USB_DIRECTION);

        telemetry.addLine("X-Drive ready.");
        telemetry.addLine("Point the robot away from the driver station before starting.");
        telemetry.update();

        waitForStart();

        // Whatever direction the robot is pointing at the start becomes "forward" for the driver.
        drive.resetYaw();

        boolean previousY = false;
        boolean previousA = false;

        while (opModeIsActive()) {
            // Stick y is negative when pushed forward, so flip it.
            double forward = shape(-gamepad1.left_stick_y);
            double right   = shape(gamepad1.left_stick_x);
            double rotate  = shape(gamepad1.right_stick_x) * TURN_SCALE;

            drive.setSpeedScale(gamepad1.left_bumper ? PRECISION_SPEED : NORMAL_SPEED);

            // Rising-edge checks so holding a button only counts once.
            if (gamepad1.y && !previousY) {
                fieldCentric = !fieldCentric;
            }
            if (gamepad1.a && !previousA) {
                drive.resetYaw();
            }
            previousY = gamepad1.y;
            previousA = gamepad1.a;

            // The right bumper temporarily forces robot-centric driving, which is the quickest way
            // out of trouble if the heading gets confused mid-match.
            boolean useFieldCentric = fieldCentric && !gamepad1.right_bumper;

            if (useFieldCentric) {
                drive.driveFieldCentric(forward, right, rotate);
            } else {
                drive.driveRobotCentric(forward, right, rotate);
            }

            telemetry.addData("Mode", useFieldCentric ? "Field-centric" : "Robot-centric");
            telemetry.addData("Speed", gamepad1.left_bumper ? "Precision" : "Normal");
            telemetry.addData("Heading", "%.1f deg", drive.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("Drive", "fwd %.2f  right %.2f  turn %.2f", forward, right, rotate);
            telemetry.addLine("Y: toggle field-centric   A: reset heading");
            telemetry.addLine("LB: precision   RB: hold for robot-centric");
            telemetry.update();
        }

        drive.stop();
    }

    /**
     * Removes the deadband and squares the stick input. Squaring keeps full power available at the
     * end of the stick's travel while making small movements much gentler.
     */
    private double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < DEADBAND) {
            return 0.0;
        }
        // Rescale so the robot starts moving right at the edge of the deadband rather than jumping.
        double scaled = (magnitude - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(input) * scaled * scaled;
    }
}
