package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Odometry Localization", group = "Sensor")
public class newLocalization extends LinearOpMode {

    // Hardware components
    private DcMotorEx verticalOdom, horizontalOdom;
    private IMU imu;

    // Position variables
    private double robotX = 0, robotY = 0, robotHeading = 0; // curr x pos for robot, curr y pos, and curr heading
    private double prevVertical = 0, prevHorizontal = 0; // previous x and previous y
    private double prevHeading = 0; // previous heading

    // Constants for odometry calculation
    private static final double TICKS_PER_REV = 2000; // GoBILDA encoder ticks per revolution, update
    private static final double WHEEL_DIAMETER = 2; // Diameter of odometry wheels (in inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private static final double WEIGHT = 1.0 / 6.0; // Weight for Simpson's Rule, shows how to weight different trajectories

    // Thresholds and constants for error-handling
    private static final double MAX_HEADING_CHANGE = Math.toRadians(10); // Max allowed change in heading (in radians)
    private static final double IMU_CALIBRATION_THRESHOLD = 0.1; // Allowable IMU drift before recalibration
    private static final double MAX_ENCODER_TICKS_PER_UPDATE = 1000; // Max allowed ticks per update to detect encoder errors

    private static int i;

    public void runOpMode() throws InterruptedException{

        // Initialize odometry wheels
        verticalOdom = hardwareMap.get(DcMotorEx.class, "backL");
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "frontR");
        imu = hardwareMap.get(IMU.class, "imu");
        i = 0;

        verticalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        // Wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            // Read current encoder values
            double currentVertical = verticalOdom.getCurrentPosition();
            double currentHorizontal = horizontalOdom.getCurrentPosition();
            // Get the robot's current heading (in radians)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Step 1: Error Handling - Check for large/rapid changes in heading (sharp turns)
            double deltaHeading = currentHeading - prevHeading;
            if (Math.abs(deltaHeading) > MAX_HEADING_CHANGE) {
                telemetry.addData("Warning", "Sharp Turn Detected - Heading Change Too Large");
                deltaHeading = 0; // Ignore large changes to avoid calculation errors
            }

            // Step 2: Error Handling - Check for outlier encoder values (noise or error in encoder data)
            double deltaVertical = currentVertical - prevVertical;
            double deltaHorizontal = currentHorizontal - prevHorizontal;


            // Convert the change in encoder ticks to inches
            deltaVertical /= TICKS_PER_INCH;
            deltaHorizontal /= TICKS_PER_INCH;

            if (Math.abs(deltaVertical) > MAX_ENCODER_TICKS_PER_UPDATE || Math.abs(deltaHorizontal) > MAX_ENCODER_TICKS_PER_UPDATE) {
                telemetry.addData("Warning", "Encoder Ticks Too Large - Possible Error Detected");
                deltaVertical = 0; // Ignore outliers to avoid incorrect position updates
                deltaHorizontal = 0;
            }

            // Step 3: Arc handling - Use average heading for relative position update
            double radiusX = 0;
            double radiusY = 0;
            if (deltaHeading != 0){
                radiusX = deltaVertical / deltaHeading;
                radiusY = deltaHorizontal / deltaHeading;
            } else {
                radiusX = deltaVertical;
                radiusY = deltaHorizontal;
            }

            double relDeltaX = radiusX * Math.sin(deltaHeading) - radiusY * (1 - Math.cos(deltaHeading));
            double relDeltaY = radiusY * Math.sin(deltaHeading) + radiusX * (1 - Math.cos(deltaHeading));

            double deltaX = prevVertical + relDeltaX * Math.cos(currentHeading) - relDeltaY * Math.sin(currentHeading);
            double deltaY = prevHorizontal + relDeltaY * Math.cos(currentHeading) + relDeltaX * Math.sin(currentHeading);

            robotX = deltaX;
            robotY = deltaY;
            robotHeading = currentHeading;


            // Step 5: IMU Calibration Handling - Detect if the IMU drifts too much
            if (Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > IMU_CALIBRATION_THRESHOLD) {
                imu.resetYaw(); // Reset IMU if it drifts beyond acceptable range
                telemetry.addData("IMU", "Recalibrating due to drift");
            }

            // Step 6: Store previous values for the next loop iteration
            prevVertical = currentVertical;
            prevHorizontal = currentHorizontal;
            prevHeading = currentHeading;

            // Output telemetry data for debugging
            telemetry.addData("X Position", robotX);
            telemetry.addData("Y Position", robotY);
            telemetry.addData("Heading", Math.toDegrees(robotHeading)); // Display heading in degrees
            telemetry.update();
        }
    }
}
