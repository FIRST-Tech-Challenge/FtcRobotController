package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

@TeleOp(name = "movement test", group = "SA_FTC")
public class TestAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private BNO055IMUImpl imu;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private double previousHeading = 0;
    private double integratedHeading = 0;

    private double currentAngle = 0;

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
    }

    private void turnRight(double power, double targetAngle) {
        while(currentAngle > -targetAngle) {
            currentAngle = getIntegratedHeading();

            System.out.println("Turning clockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);
            setPower(0,0, power);
        }

        setPower(0, 0, 0);
        sleep(100); // Resting
    }

    private void turnLeft(double power, double targetAngle) {
        while(currentAngle < -targetAngle) {
            currentAngle = getIntegratedHeading();

            System.out.println("Turning clockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);
            setPower(0,0, -power);
        }

        setPower(0, 0, 0);
    }

    private void turn(double power, double targetAngle) {
        boolean shouldTurnClockwise = getShortestDirection(currentAngle, targetAngle);

        if (shouldTurnClockwise) { // Turn clockwise
            turnLeft(power, targetAngle);
        } else { // Turn counterclockwise
            turnRight(power, targetAngle);
        }
    }

    // Check if the target angle is reached
    // private boolean isAngleReached(double targetAngle, boolean clockwise) {
    //     if (clockwise) {
    //         return currentAngle >= targetAngle;
    //     } else {
    //         return currentAngle <= targetAngle;
    //     }
    // }

    // Return true if shortest direction is clockwise, and false if shortest direction is counterclockwise
    public boolean getShortestDirection(double currentAngle, double targetAngle) {
        double angleDifference = (targetAngle - currentAngle + 360) % 360;
        System.out.println(angleDifference);

        if (angleDifference > 180) {
            return false;
        } else {
            return true;
        }
    }

    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
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
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        imu.initialize(parameters);

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

            currentAngle = imu.getAngularOrientation().firstAngle;

            //setPowerWithTime(0.5, 0, 0, 1);

            //turn(0.5, 100);
            //turn(0.5, 90);
            //System.out.println("shimshon");

            //turn(0.5, 100);
            //turn(0.5, -90);

            turnRight(0.5, 90);
            turnRight(0.5, 270);

            break;
            //System.out.println(currentAngle);
            //setPowerWithTime(0.5, 0, 0, 1);
        }
    }
}