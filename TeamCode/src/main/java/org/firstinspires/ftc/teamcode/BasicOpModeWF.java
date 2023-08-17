package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Basic: Omni TeleOp", group="Linear Opmode")
public class BasicOpModeWF extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private IMU imu = null;


    private void Initialize()
    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


    }

    @Override
    public void runOpMode() throws InterruptedException{
        Initialize();
        waitForStart();

        runtime.reset();
        imu.resetYaw();
        int speedFactor = 5; // it goes from 1 (10% speed) to 10 (100% speed)


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double yaw = gamepad1.right_stick_x;
            if(gamepad1.right_bumper)
                speedFactor = 10;
            if(gamepad1.left_bumper)
                speedFactor = 5;
            if(gamepad1.back)
                imu.resetYaw();
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if(gamepad1.dpad_up)
                    y = 1.0;
                if(gamepad1.dpad_down)
                    y = -1.0;
                if (gamepad1.dpad_left)
                    x = -1.0;
                if (gamepad1.dpad_right)
                    x = 1.0;

                double newx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double newy = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                x = newx;
                y = newy;



            }



            SendPowerToWheels(x, y, yaw, speedFactor);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X/Y/Yaw", "%4.2f, %4.2f, %4.2f", x, y, yaw);
            telemetry.addData("SpeedFactor = ", "%d", speedFactor);
            telemetry.addData("BotHeading = ", "%4.2f", botHeading);
            telemetry.update();
        }
    }

    private void SendPowerToWheels(double x, double y, double yaw, int speedFactor )
    {
        double max;
        double leftFrontPower  = y + x + yaw;
        double rightFrontPower = y - x - yaw;
        double leftBackPower   = y - x + yaw;
        double rightBackPower  = y + x - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * speedFactor / 10.0);
        rightFrontDrive.setPower(rightFrontPower * speedFactor / 10.0);
        leftBackDrive.setPower(leftBackPower * speedFactor / 10.0);
        rightBackDrive.setPower(rightBackPower * speedFactor / 10.0);
    }
}
