package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="PcontrollerProblem")

public class PcontrollerProblem extends LinearOpMode {
    public Hardware robot = new Hardware();
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        ElapsedTime e = new ElapsedTime();
        double origAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // The starting angle of the robot

        // Repeat until the stop button is pressed
        do {
            // Wait for error to accumulate (using ElapsedTime)
            e.reset();
            sleep(20);

            // Gather data
            double newAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double time = e.milliseconds();

            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle); // For telemetry
            telemetry.update();

            /* Your code goes here. It should probably look something like this:

            1. Calculate the error between the new angle and the old angle
            2. Multiply the error by a coefficient to get the correction factor
            3. Set the motor powers appropriately by adding/subtracting the correction factor

             */
            double error = origAngle - newAngle;
            double corection = error * .5;

            if (corection < 0) {
                robot.m0.setPower(.5);
                robot.m1.setPower(.5+corection);
                robot.m2.setPower(.5);
                robot.m3.setPower(.5+corection);
            } else {
                robot.m0.setPower(.5-corection);
                robot.m1.setPower(.5);
                robot.m2.setPower(.5-corection);
                robot.m3.setPower(.5);
            }
        } while (!isStopRequested()) ;
    }
}