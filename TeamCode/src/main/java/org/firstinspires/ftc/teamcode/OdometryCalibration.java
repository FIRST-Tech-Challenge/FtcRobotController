package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


//NOT UPDATED


@TeleOp(name="Odometry Calibration", group="Odometry")
public class OdometryCalibration extends LinearOpMode {
    HardwareMapV2 robot = null;
    BNO055IMU imu;
    Drivetrain drivetrain;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.setEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Odometry System Calibration Status", "Initialized Wheels");
        telemetry.update();

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.update();

        waitForStart();

        while(getZAngle() < 90 && opModeIsActive()){
            drivetrain.forward(0.5);
            if(getZAngle() < 60) {
                drivetrain.spin(true, 0.5);
            }else{
                drivetrain.spin(true, 0.3);
            }
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }
        drivetrain.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        double verticalEncoderTickOffsetPerDegree = robot.leftVertical.getCurrentPosition() - robot.rightVertical.getCurrentPosition();
        double leftWheelDistance = (robot.leftVertical.getCurrentPosition()*180)/(360*Math.PI);
        double rightWheelDistance = (robot.rightVertical.getCurrentPosition()*180)/(360*Math.PI);

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
//            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
//            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", robot.leftVertical.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.rightVertical.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);
            telemetry.addData("Left Wheel Radius", leftWheelDistance);
            telemetry.addData("Right Wheel Radius", rightWheelDistance);

            //Update values
            telemetry.update();
        }

    }

    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }
}
