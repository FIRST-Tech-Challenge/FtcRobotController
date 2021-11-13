package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Enhancement.Robot;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

import java.io.IOException;


/**
 * OmniDirectional Drive allows the robot to be operated in third person rather than first person.
 */
@TeleOp(name = "OmniDirectionalDrive", group = "Assisted Driving")
public class OmniDirectionalDrive extends LinearOpMode {
    private final QuickTelemetry quickTelemetry = new QuickTelemetry(telemetry);
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double power = .30;
    private Robot robot;

    private void initOpMode() throws IOException {
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer, false);
    }

    /**
     * Called when init button is pressed.
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException because it might need to throw this when the op mode needs to be terminated early.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        // wait for start button.
        waitForStart();

        quickTelemetry.telemetry("Mode", "running");

        // wait for 1 second
        sleep(1000);


        while (opModeIsActive()) {
            // Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = -gamepad1.right_stick_y;

//            // Get robot angle
//            double robotAngle = imu.getAngularOrientation().firstAngle;
//            double robotAngle360 = to360(robotAngle);

            // Synthesize robot angle
            double robotAngle = Math.atan2(rightStickY, rightStickX) - Math.PI / 2;
            double robotAngle360 = to360(robotAngle);

            // Get controller angle
            double controllerAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 2;
            double controllerAngle360 = to360(controllerAngle);

            // Find angle between controller and robot
            double angleBetween = smallestAngleBetween(robotAngle360, controllerAngle360) * findRotationDirection(robotAngle360, controllerAngle360);
            double angleBetween360 = to360(angleBetween);

            // Convert angle to X and Y
            double correctedX = Math.cos(angleBetween360 + Math.PI / 2);
            double correctedY = Math.sin(angleBetween360 + Math.PI / 2);

            // Drive the motors
            Drive drive = robot.drive;
            double[] powers = drive.calcMotorPowers(rightStickX, correctedY, 0);
            drive.rearLeft.setPower(powers[0]);
            drive.frontLeft.setPower(powers[1]);
            drive.rearRight.setPower(powers[2]);
            drive.frontRight.setPower(powers[3]);

            quickTelemetry.telemetry("Fake RBT Angle", ((Double) robotAngle360).toString());
            quickTelemetry.telemetry("CONTROL Angle", ((Double) controllerAngle360).toString());


            resetAngle();
        }

        // turn the motors off.
        stopMotors();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */


    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Stops the motors
     * <p>
     * uses {@link com.qualcomm.robotcore.hardware.DcMotorEx#setPower(double)}
     *
     * @see Drive
     * @see com.qualcomm.robotcore.hardware.DcMotorEx#setPower(double)
     */
    private void stopMotors() {
        robot.drive.frontLeft.setPower(0);
        robot.drive.frontRight.setPower(0);
        robot.drive.rearLeft.setPower(0);
        robot.drive.rearRight.setPower(0);
    }

    /**
     * Converts from euler units to 360 degrees
     * Goes from 0 to 360 in a clockwise fashion
     * Accepts numbers between -180 and 180
     */
    private double to360(double angle) {
        if (angle >= 0) {
            return angle;
        } else {
            return angle + 360;
        }
    }

    /**
     * Returns the smallest angle between angle1 and angle 2
     * Accepts the range 0 - 360 for both angles.
     * <p>
     * It checks whether 360 - (angle2 - angle 1) or angle 2 - angle 1 is the smallest.
     *
     * @param angle1 first angle
     * @param angle2 second angle
     * @return smallest angle
     */
    private double smallestAngleBetween(double angle1, double angle2) {
        double distanceBetween = Math.abs(angle2 - angle1);
        return Math.min((360 - distanceBetween), distanceBetween);
    }

    /**
     * Determines the shortest way to rotate to the goal angle
     * Accepts angles from 0 - 360 for both inputs
     *
     * @param robot The robot
     * @param goal  The goal
     * @return the shortest way to rotate to the goal angle
     */
    private double findRotationDirection(double robot, double goal) {
        double i;
        if (robot <= 180) {
            if (goal < robot || goal > robot + 180) {
                i = -1;
            } else {
                i = 1;
            }
        } else {
            if (goal > robot || goal < robot - 180) {
                i = 1;
            } else {
                i = -1;
            }
        }

        return i;
    }
}
