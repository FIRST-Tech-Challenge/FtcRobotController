package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Variables.*;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name ="MarkAuto", group = "A+")
public class marksAutonomous extends DriveMethods {
    BNO055IMU imu;
    double previousZ = 0.0;
    double integratedZ = 0.0;
    double linearSlideClicks = 537.7;
    double motorCircumference = .112;
    DcMotor motorLinearSlide;

    boolean calibrated = false;

    @Override
    public void runOpMode() {

        initMotorsBlue();

        motorLinearSlide = hardwareMap.get(DcMotor.class,"motorLS");
        motorLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        calibrateImu();

        waitForStart();

        LinearSlidePosition(.2);


        while(opModeIsActive()) {

        }
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        ROTATE_LEFT,
        ROTATE_RIGHT,
        RIGHT,
        LEFT,

    }

    public void driveDirection(Direction direction, double power) {
        switch(direction) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;

            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;

            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                break;

            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                break;

            case ROTATE_LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;

            case ROTATE_RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
        }
    }

    public void driveForTime (int seconds, double power,Direction direction) { //seconds: 10, power 1, direction: Forward

        driveDirection(direction, power);


        sleep(seconds * 1000);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    public void driveForDistance (double distance, double power, Direction direction) {
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        int targetClicks;
        targetClicks = (int)(distance * rotationsPerMeter * clicksPerRotation);
        int currentClicks = 0;

        motorBL.setTargetPosition((targetClicks));
        motorFL.setTargetPosition((targetClicks));
        motorBR.setTargetPosition((targetClicks));
        motorFR.setTargetPosition((targetClicks));

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        while(currentClicks < targetClicks) {
            driveDirection(direction, power);
            currentClicks = (Math.abs(motorBL.getCurrentPosition()) +
            Math.abs(motorFL.getCurrentPosition()) +
            Math.abs(motorBR.getCurrentPosition()) +
            Math.abs(motorFR.getCurrentPosition()))/4;
            telemetry.addLine("Current Distance: " + currentClicks/clicksPerRotation/rotationsPerMeter);
            telemetry.update();
        }

        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);

    }

    public void calibrateImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        calibrated = true;
    }

    public double GetCurrentZ() {
        double currentZ;
        if (calibrated == false) {
            calibrateImu();
        }

        Orientation CurrentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentZ = CurrentAngle.firstAngle;
        return currentZ;
    }

    public double CummulativeZ() {
       double currentZ = GetCurrentZ();
       double deltaZ = currentZ - previousZ;

       if (deltaZ < -180) {
           deltaZ += 360;
       }

       else if (deltaZ >= 180) {
           deltaZ -= 360;
       }

       integratedZ += deltaZ;
       previousZ = currentZ;

       return integratedZ;
    }

    public void LinearSlidePosition(double meters) {
        int targetEncoderClicks = 0;
        int currentPosition = 0;
        int dif = 0;



        if (meters < 0.5) {
            motorLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetEncoderClicks = (int)(meters*linearSlideClicks/motorCircumference);
            currentPosition = motorLinearSlide.getCurrentPosition();
            dif = targetEncoderClicks - currentPosition;
            while(Math.abs(dif) > 20) {
                telemetry.addLine("Dif: " + dif);
                telemetry.addLine("Current position: " + currentPosition);
                telemetry.addLine("Target position: " + targetEncoderClicks);
                telemetry.addLine("Power" + ((dif / 3000) + 0.05));
                telemetry.update();

                currentPosition = motorLinearSlide.getCurrentPosition();
                dif = targetEncoderClicks - currentPosition;
                motorLinearSlide.setPower((dif / 3000) + 0.05);
            }
        }

    }
}
