
package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="field Centric huh?", group="Linear OpMode")
//@Disabled
public class FieldCentricDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    protected MecanumDrive drive;
    private double  yMultiplier = 1;
    private IMU imu;
    private Pose2d startingPos = new Pose2d(0,0,Math.toRadians(90));

    double posX = drive.pose.position.x;
    double posY = drive.pose.position.y;
    double leftY;
    double leftX;
    double rightX;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        // region Hardware Initialization

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        drive = new MecanumDrive(hardwareMap, startingPos);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            Pose2d poseEstimate = drive.pose;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Vector2d input = new Vector2d(-currentGamepad1.left_stick_x * yMultiplier,
                    currentGamepad1.left_stick_y * yMultiplier);
                    //.times(Math.toRadians(poseEstimate.heading.toDouble()));
                    //highly sus code above
                    //this is replica of the rotate code from last year
            Vector2d rotated = rotatedVector(drive.pose.heading.toDouble());

            if (currentGamepad1.options && !previousGamepad1.options) {
                if (yMultiplier == 1) {
                    yMultiplier = -1;
                } else {
                    yMultiplier = 1;
                }
            }
            if (currentGamepad1.start && !previousGamepad1.start) {
                posX = drive.pose.position.x;
                posY = drive.pose.position.y;
               drive.pose =  new Pose2d(posX,posY,Math.toRadians(90));
            }
            double leftY = -gamepad1.left_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-drive.pose.heading.toDouble()) - leftY * Math.sin(-drive.pose.heading.toDouble());
            double rotY = leftX * Math.sin(-drive.pose.heading.toDouble()) + leftY * Math.cos(-drive.pose.heading.toDouble());

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
            double frontLeftPower = (rotY + rotX + rightX) / denominator;
            double backLeftPower = (rotY - rotX + rightX) / denominator;
            double frontRightPower = (rotY - rotX - rightX) / denominator;
            double backRightPower = (rotY + rotX - rightX) / denominator;

            drive.leftFront.setPower(frontLeftPower);
           drive.leftBack.setPower(backLeftPower);
            drive.rightFront.setPower(frontRightPower);
           drive.rightBack.setPower(backRightPower);


            // Update everything. Odometry. Etc.
            drive.updatePoseEstimate();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("angleIMU", robotOrientation);
            telemetry.addData("angle odometry",drive.pose.heading.toDouble());
            telemetry.update();
        }
    }
    private Vector2d rotatedVector(double angle){
        double x = -currentGamepad1.left_stick_x * yMultiplier;
        double y = currentGamepad1.left_stick_y * yMultiplier;
        double newX = x * cos(angle) - y * sin(angle);
        double newY = x * sin(angle) + y * cos(angle);
        return new Vector2d(newX,newY);
    }

}
