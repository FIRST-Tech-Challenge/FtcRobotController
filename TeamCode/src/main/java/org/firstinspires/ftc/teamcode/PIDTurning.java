package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PIDTurning", group="Pushbot")
public class PIDTurning extends LinearOpMode {

    Orientation lastAngles = new Orientation();
    double globalAngle;


    public DMHardware robot = new DMHardware(true);  // We are using encoders, so pass it ????
    private ElapsedTime runtime = new ElapsedTime();
    //instantiating variable imu of type BNO055IMU
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        //call imu.initialize() and feed it parameters
        imu.initialize(parameters);
        waitForStart();


        resetAngle();
        turnWithPID(90, 0.085, 0, 0.0001, 1);


    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }
    public void turnWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        double error = targetAngle - getHeading();
        double sum = error;
        double previousError = error;
        double correction = 0;
        double slope = 0;
        runtime.reset();
        while (Math.abs(error) > threshold && opModeIsActive()) {
            telemetry.addData("error", error);
            telemetry.update();
            error = targetAngle - getHeading();
            slope = (error - previousError) / (double) runtime.time();
            runtime.reset();
            sum += error;
            correction = kp * error + ki * sum + kd * slope;


            robot.leftMotor.setPower(correction);
            robot.rightMotor.setPower(-1 * correction );
        }
    }
}