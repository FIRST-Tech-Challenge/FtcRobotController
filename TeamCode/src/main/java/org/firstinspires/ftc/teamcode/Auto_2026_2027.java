package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto_2026_2027", group="Robot")
public class Auto_2026_2027 extends OpMode {

    //---Declare OpMode members.---//
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor flywheel;
    private DcMotor transfer;
    private DcMotor intake;
    // IMU Stuff
    IMU imu;
    private double yaw;
    private ElapsedTime runtime = new ElapsedTime();

    GoBildaPinpointDriver pinpoint;

    @Override
    public void init()
    {
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorBR");

        // IMU - FIELD CENTRIC STUFF!! //
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // NOTE: If the logo of control hub that says "Control Hub" is facing right ex: robot is facing forward and its on its right side. . then tell the firectiong its facing right.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Defining stuff frfr.. I think? - Brayyten
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Initializing the IMU's orientation..
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop()
    {
        if(gamepad1.a){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        displayIMU();

        // telememtery
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
    }

    public void displayIMU()
    {
        {
            //telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.options) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press options (menu) on Gamepad to reset\n");
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            // Retrieve Rotational Angles and Velocities for IMU
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // field centric
            yaw = orientation.getYaw(AngleUnit.RADIANS);

            // IMU Telemetry
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
    public void configurePinpoint(){

        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    public void auto_drivebase(){
        pinpoint.update();

        double robotXDestination = 1;
        double robotYDestination = 1;

        Pose2D pose2D = pinpoint.getPosition();
        double robotXPosition = pose2D.getX(DistanceUnit.INCH);
        double robotYPosition = pose2D.getY(DistanceUnit.INCH);

        // Get error
        double robotPinpointXError = robotXDestination - robotXPosition;
        double robotPinpointYError = robotYDestination - robotYPosition;

        // Field Centric stuff
        double x_Mod = robotPinpointYError * Math.cos(yaw) + robotPinpointXError * Math.sin(yaw);
        double y_Mod = robotPinpointXError * Math.cos(yaw) - robotPinpointYError * Math.sin(yaw);

        frontRightDrive = y_Mod;
        backRightDrive = y_Mod;
        frontLeftDrive = x_Mod;
        backLeftDrive = x_Mod;
        // NOTE FOR TMR: PUT MOTORS IN HARWARE MAP

        // Telemetry
        telemetry.addData("X Error", robotPinpointXError);
        telemetry.addData("Y Error", robotPinpointYError);
    }
}
