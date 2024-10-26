/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class localization {

    @TeleOp(name = "Odometry Localization", group = "Localization")
    public class OdometryLocalization extends LinearOpMode {



        // hardware components
        private DcMotor verticalOdom;
        private DcMotor horizontalOdom;
        private BNO055IMU imu;

        // position variables
        private double xPosition = 0;  // horizontal movement
        private double yPosition = 0;  // vertical movement
        private double heading = 0;    // orientation angle (from IMU)

        // constants for calculating distance
        private static final double TICKS_PER_REV = 8192;  // GoBILDA encoder ticks per revolution
        private static final double WHEEL_DIAMETER = 2;    // diameter of odometry wheels (in inches)
        private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

        @Override
        public void runOpMode() {
            // initialize hardware
            verticalOdom = hardwareMap.get(DcMotor.class, "verticalOdom");
            horizontalOdom = hardwareMap.get(DcMotor.class, "horizontalOdom");
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            // initialize IMU
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            // reset encoder values
            verticalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontalOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            // tracking time
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();

            // variables for previous encoder values
            int prevVerticalTicks = 0;
            int prevHorizontalTicks = 0;

            while (opModeIsActive()) {
                // get the current position of the odometry pods
                int currentVerticalTicks = verticalOdom.getCurrentPosition();
                int currentHorizontalTicks = horizontalOdom.getCurrentPosition();

                // calculate the change in encoder values
                int deltaVertical = currentVerticalTicks - prevVerticalTicks;
                int deltaHorizontal = currentHorizontalTicks - prevHorizontalTicks;

                // update the previous ticks for the next loop
                prevVerticalTicks = currentVerticalTicks;
                prevHorizontalTicks = currentHorizontalTicks;

                // convert encoder ticks to inches
                double deltaY = deltaVertical / TICKS_PER_INCH;
                double deltaX = deltaHorizontal / TICKS_PER_INCH;

                // get the robot's heading from the IMU
                heading = imu.getAngularOrientation().firstAngle;

                // update robot's position using trigonometry (accounting for rotation)
                xPosition += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
                yPosition += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

                // output position and heading telemetry
                telemetry.addData("X Position", xPosition);
                telemetry.addData("Y Position", yPosition);
                telemetry.addData("Heading", Math.toDegrees(heading));  // Convert to degrees for readability
                telemetry.update();
            }
        }
    }
}*/

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Odometry Localization", group = "Sensor")
public class localization extends LinearOpMode {


    // hardware
    private DcMotorEx verticalOdom, horizontalOdom;
    private IMU imu;

    // variables to track position
    private double robotX = 0, robotY = 0;
    private double prevVertical = 0, prevHorizontal = 0;
    private double prevHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize odometry wheels
        verticalOdom = hardwareMap.get(DcMotorEx.class, "verticalOdom");
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "horizontalOdom");
        imu = hardwareMap.get(IMU.class, "imu");

        verticalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        // wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // read current encoder values
            double verticalPos = verticalOdom.getCurrentPosition();
            double horizontalPos = horizontalOdom.getCurrentPosition();

            // read current IMU heading
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // calculate the changes in the odometry
            double dVertical = verticalPos - prevVertical;
            double dHorizontal = horizontalPos - prevHorizontal;
            double dHeading = heading - prevHeading;

            // update position based on movement
            robotX += dVertical * Math.cos(heading) - dHorizontal * Math.sin(heading);
            robotY += dVertical * Math.sin(heading) + dHorizontal * Math.cos(heading);

            // store current values for the next loop
            prevVertical = verticalPos;
            prevHorizontal = horizontalPos;
            prevHeading = heading;

            // output telemetry data
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Odometry Localization", group = "Sensor")
public class localization extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize odometry wheels
        verticalOdom = hardwareMap.get(DcMotorEx.class, "backL");
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "frontR");
        imu = hardwareMap.get(IMU.class, "imu");

        verticalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        // Wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
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


            // Step 3: Arc handling - Use average heading for position update
            double radiusX;
            double radiusY;
            if (deltaHeading != 0){
                radiusX = deltaVertical / deltaHeading;
                radiusY = deltaHorizontal / deltaHeading;
            } else {
                radiusX = deltaVertical;
                radiusY = deltaHorizontal;
            }
            double relDeltaX = radiusX * Math.sin(deltaHeading) - radiusY * (1 - Math.cos(deltaHeading));
            double relDeltaY = radiusY * Math.sin(deltaHeading) + radiusX * (1 - Math.cos(deltaHeading));

            double headingAverage = (prevHeading + currentHeading) / 2.0;
            double deltaX = relDeltaX * Math.cos(currentHeading) - relDeltaY * Math.sin(currentHeading) + prevVertical;
            double deltaY = relDeltaY * Math.cos(currentHeading) + relDeltaX * Math.sin(currentHeading) + prevHorizontal;

            // Update robot's global position
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

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Odometry Localization", group = "Sensor")
public class localization extends LinearOpMode {

    // hardware
    private DcMotorEx verticalOdom, horizontalOdom;
    private IMU imu;

    // position tracking
    private double robotX = 0, robotY = 0;
    private double prevVertical = 0, prevHorizontal = 0;
    private double prevHeading = 0;

    // small threshold to prevent tiny movements from accumulating
    private static final double MOVEMENT_THRESHOLD = 1e-3;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize odometry wheels and IMU
        verticalOdom = hardwareMap.get(DcMotorEx.class, "backL");
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "frontL");
        imu = hardwareMap.get(IMU.class, "imu");

        // reset encoders and IMU yaw
        verticalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        // wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // read encoder values
            double verticalPos = verticalOdom.getCurrentPosition();
            double horizontalPos = horizontalOdom.getCurrentPosition();

            // read current IMU heading (in radians)
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // calculate changes in odometry values
            double dVertical = verticalPos - prevVertical;
            double dHorizontal = horizontalPos - prevHorizontal;
            double dHeading = heading - prevHeading;

            // check if changes are significant to avoid noise accumulation
            if (Math.abs(dVertical) > MOVEMENT_THRESHOLD || Math.abs(dHorizontal) > MOVEMENT_THRESHOLD || Math.abs(dHeading) > MOVEMENT_THRESHOLD) {
                // update position based on movement
                robotX += dVertical * Math.cos(heading) - dHorizontal * Math.sin(heading);
                robotY += dVertical * Math.sin(heading) + dHorizontal * Math.cos(heading);
            }

            // store current values for the next loop
            prevVertical = verticalPos;
            prevHorizontal = horizontalPos;
            prevHeading = heading;

            // output telemetry data
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}*/

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Odometry Localization", group = "Sensor")
public class localization extends LinearOpMode {

    // hardware
    private DcMotorEx verticalOdom, horizontalOdom;
    private IMU imu;

    // variables to track position
    private double robotX = 0, robotY = 0;
    private double prevVertical = 0, prevHorizontal = 0;
    private double prevHeading = 0;

    // Constants for scaling (recheck these values with real-world measurements if necessary)
    private static final double WHEEL_DIAMETER_INCHES = 2.0;  // Adjust if your odometry wheel diameter differs
    private static final double TICKS_PER_REV = 2000; // or 2048      // GoBILDA encoder
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize odometry wheels
        verticalOdom = hardwareMap.get(DcMotorEx.class, "backL"); // Map vertical odom to backL
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "frontL"); // Map horizontal odom to frontL
        imu = hardwareMap.get(IMU.class, "imu");

        // Reset encoders and IMU
        verticalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        // wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Read current encoder values
            double verticalPos = verticalOdom.getCurrentPosition();
            double horizontalPos = horizontalOdom.getCurrentPosition();

            // Read current IMU heading in radians
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Calculate the changes in the odometry values
            double dVertical = (verticalPos - prevVertical) / TICKS_PER_INCH;  // Convert ticks to inches
            double dHorizontal = (horizontalPos - prevHorizontal) / TICKS_PER_INCH; // Convert ticks to inches

            // Update the X and Y positions based on heading and encoder changes
            robotX += dVertical * Math.cos(heading) - dHorizontal * Math.sin(heading);
            robotY += dVertical * Math.sin(heading) + dHorizontal * Math.cos(heading);

            // Update previous values for the next loop
            prevVertical = verticalPos;
            prevHorizontal = horizontalPos;
            prevHeading = heading;

            // Output telemetry data
            telemetry.addData("X Position (in)", robotX);
            telemetry.addData("Y Position (in)", robotY);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));  // Convert heading to degrees for easier understanding
            telemetry.update();
        }
    }
}
*/
