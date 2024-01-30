package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "movement test", group = "SA_FTC")
//@Autonomous
public class TestAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    //change these values for our motors
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private int part = 0;
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

    private double calculatePower(double minPower, double maxPower, double startAngle, double targetAngle, double c, double currentPower) {
        double slowDownAngle = targetAngle - c;


        double accelerationPower = minPower + ((currentAngle - startAngle) / c) * (maxPower - minPower);
        double slowDownPower = (currentAngle - slowDownAngle) * maxPower / c;
        double retPower = 0;



        if (part < 2 && currentAngle < startAngle + c){
            part = 1;
            retPower = accelerationPower;
        }
        else if (part < 3 && currentAngle > startAngle + c && currentAngle < slowDownAngle) {
            part = 2;
            retPower = maxPower;
        } else if (currentAngle < targetAngle) {
            part = 3;
            retPower = Math.min( Math.max(currentPower, 0.1), slowDownPower);
        }

        telemetry.addData("current angle", currentAngle);
        telemetry.addData("current part", part);
        telemetry.update();
        return retPower;
    }

    private void turnRight(double minPower, double maxPower, double targetAngle, double c) {
        currentAngle = getIntegratedHeading();

        double startAngle = currentAngle;
        double power = 0;


        while (currentAngle < targetAngle) {
            currentAngle = getIntegratedHeading();

            // for debugging
            //System.out.println("Turning clockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);

            power = calculatePower(minPower, maxPower, startAngle, targetAngle, c, power);

            setPower(0, 0, power);
        }

        setPower(0, 0, 0);
        sleep(2000); // Resting

    }

    private void turnLeft(double minPower, double maxPower, double targetAngle, double c) {
        currentAngle = getIntegratedHeading();
        double startAngle = currentAngle;
        double power = 0;


        while (currentAngle > targetAngle) {
            currentAngle = getIntegratedHeading();

            // for debugging
            //System.out.println("Turning counterclockwise. Current angle: " + currentAngle + ", Target: " + targetAngle);

            power = calculatePower(minPower, maxPower, startAngle, targetAngle, c, power);

            setPower(0, 0, -power);
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

        telemetry.addData("current angle", -integratedHeading);
        telemetry.update();

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
        //turnRight(0.5, 180);
        //turnRight(0.5, 270);
        //turnRight(0.5, 360);
        //turnLeft(0.5, 180);
        //turnLeft(0.5, 0);

        turnRight(0.3, 0.55, 360, 25);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftMotor.setTargetPosition(newLeftTarget);
            frontRightMotor.setTargetPosition(newRightTarget);
            backLeftMotor.setTargetPosition(newLeftTarget);
            backRightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }

    @Override
    public void runOpMode() {
        // Set motors
        //frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        //backLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        //frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        //backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
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

            unitTestTurn();

            telemetry.addData("current angle", currentAngle);
            telemetry.update();

            //break;
            //System.out.println(currentAngle);
            //setPowerWithTime(0.5, 0, 0, 1);
        }
    }
}
