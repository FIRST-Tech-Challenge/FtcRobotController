package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.BNO055IMUImpl;

@TeleOp(name = "movement test", group = "SA_FTC")
public class TestAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private double previousHeading = 0;
    private double integratedHeading = 0;

    private double currentAngle = 0;

    private final boolean debug = true;

    private void setPower(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }

    private void setPowerWithTime(double axial, double lateral, double yaw, double time) {
        double startTime = runtime.seconds();

        while (runtime.seconds() < startTime + time) {
            setPower(axial, lateral, yaw);
        }

        setPower(0, 0, 0);
        sleep(100); // Resting
    }

    private void turnRight(double minPower, double maxPower, double targetAngle, double percentage) {
        double m1 = (maxPower - minPower) / percentage;
        double m2 = maxPower / -percentage;
        double p1 = targetAngle * percentage / 100;
        double p2 = targetAngle * (100 - percentage) / 100;

        double power = 0;
        while(currentAngle < targetAngle) {
            currentAngle = getIntegratedHeading();

            // for debugging
            //System.out.println("Turning clockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);
            telemetry.addData("current angle", currentAngle);
            telemetry.update();

            if (currentAngle < p1){
                power = m1 * currentAngle + minPower;
            }

            if (currentAngle > p1 && currentAngle < p2) {
                power = maxPower;
            }

            if (currentAngle > p2){
                power = m2 * currentAngle - m2 * 100;
            }

            setPower(0,0, power);
        }

        setPower(0, 0, 0);
        sleep(2000); // Resting
    }

    private void turnLeft(double minPower, double maxPower, double targetAngle, double percentage) {
        while(currentAngle > targetAngle) {
            currentAngle = getIntegratedHeading();
            // for debugging
            if (debug) {
                //System.out.println("Turning clockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);
                telemetry.addData("current angle", currentAngle);
                telemetry.update();
            }
            setPower(0,0, -power);
        }

        setPower(0, 0, 0);
        sleep(2000); // Resting
    }

    // transform the angles from (180,-179) to (inf, -inf)
    private double getIntegratedHeading() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //double currentHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;

        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return -integratedHeading;
    }

    private void unitTestTurn() {
        //turnLeft(0.5, -90);
        //turnLeft(0.5, -110);

        //turnRight(0.5, 100);
        //turnLeft(0.5, 75);

        //turnLeft(0.5, -90);
        //turnLeft(0.5, -180);

        //turnLeft(0.5, -279);
        //turnRight(0.5, 73);

        //turnLeft(0.5,-155);

        //turnRight(0.5, 90);
        turnRight(0.5, 180);
        turnRight(0.5, 270);
        turnRight(0.5, 360);
        turnLeft(0.5, 180);
        turnLeft(0.5, 0);
    }

    @Override
    public void runOpMode() {
        // Set motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb));
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        //parameters.loggingEnabled = true;   //For debugging
        //parameters.loggingTag = "IMU";      //For debugging
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // System.out.print(imu.getAngularOrientation());
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            currentAngle = getIntegratedHeading();

            unitTestTurn();

            //break;
            //System.out.println(currentAngle);
            //setPowerWithTime(0.5, 0, 0, 1);
        }
    }
}
