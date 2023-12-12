package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//make sure to try driveFieldCentricFor and driveFieldCentric to make sure your timer works
//if For doesn't work, figure out how to send commands in sequence
@Autonomous(name = "Autonomous-toolbox")
public class Auton_BHI260 extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        driveFieldCentricFor(1, 0, 0, 3000, timer);
    }

    public static double normalize(double degrees) {
        double normalized_angle = AngleUnit.normalizeDegrees(degrees);
        if (normalized_angle > 180) {
            normalized_angle -= 360;
        }
        return normalized_angle;
    }

    public double getAngle() {
        //Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles =
                gyro.getRobotOrientation(
                        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // ZYX is Original
        return angles.firstAngle;
    }

    // Drive or Strafe to at some power while turning to some angle.
    public void driveFieldCentricFor(double drive, double angle, double strafe, double milliseconds, ElapsedTime timer) {
        timer.reset();
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        double topRightPow, backRightPow, topLeftPow, backLeftPow;
        //double botHeading = -Math.toRadians(gyro.getAngularOrientation().firstAngle);
        double botHeading = -Math.toRadians(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); //is this the same as above?

        // Compute how much you need to turn to maintain that angle
        double currAngle = getAngle();
        double angleDiff = normalize(currAngle - angle);
        double turn = Range.clip(angleDiff * 0.01, -1, 1) * 1.1; // multiply by some constant
        // so the robot turns smoothly


        double rotX = drive * Math.cos(botHeading) - strafe * Math.sin(botHeading);
        double rotY = drive * Math.sin(botHeading) + strafe * Math.cos(botHeading);

        // Do the math found in GM0
        double denominator = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);
        topLeftPow = (rotY + rotX + turn) / denominator;
        backLeftPow = (rotY - rotX + turn) / denominator;
        topRightPow = (rotY - rotX - turn) / denominator;
        backRightPow = (rotY + rotX - turn) / denominator;

        while (timer.milliseconds() <= milliseconds) {
            setDrivePowers(backLeftPow, topLeftPow, backRightPow, topRightPow);
        }
    }

    public void driveFieldCentric(double drive, double angle, double strafe) {
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        double topRightPow, backRightPow, topLeftPow, backLeftPow;
        double botHeading = -Math.toRadians(gyro2.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); //is this the same as above?

        // Compute how much you need to turn to maintain that angle
        double currAngle = getAngle();
        double angleDiff = normalize(currAngle - angle);
        double turn = Range.clip(angleDiff * 0.01, -1, 1) * 1.1; // multiply by some constant
        // so the robot turns smoothly


        double rotX = drive * Math.cos(botHeading) - strafe * Math.sin(botHeading);
        double rotY = drive * Math.sin(botHeading) + strafe * Math.cos(botHeading);

        // Do the math found in GM0
        double denominator = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);
        topLeftPow = (rotY + rotX + turn) / denominator;
        backLeftPow = (rotY - rotX + turn) / denominator;
        topRightPow = (rotY - rotX - turn) / denominator;
        backRightPow = (rotY + rotX - turn) / denominator;

        setDrivePowers(backLeftPow, topLeftPow, backRightPow, topRightPow);
    }

    public void setDrivePowers(double backLeftPow, double topLeftPow, double backRightPow, double topRightPow) {
        backLeft.setPower(backLeftPow);
        topLeft.setPower(topLeftPow);
        backRight.setPower(backRightPow);
        topRight.setPower(topRightPow);
    }
}
