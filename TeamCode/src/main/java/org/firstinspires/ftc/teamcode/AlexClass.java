package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Variables.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.graphics.drawable.GradientDrawable;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous     (name="AlexClass", group = "A")

public class AlexClass extends DriveMethods {

//    boolean imuIsCali = false;
//    double prevHead = 0;
//    double inteHead = 0;
//    BNO055IMU imu;
    boolean inSlideLoop = false;
    int target;
    int dif;

    public void runOpMode() {
        motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");

        waitForStart();

        GoToHeight(1950);


        while (opModeIsActive()) {
            GoToHeight(100);
            GoToHeight(1950);
//            GoToHeight(1950);
//            if (inSlideLoop) {
//                if ((Math.abs(dif) >= 150)) {
//                    telemetry.addLine(dif + "..difference");
//                    telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
//                    telemetry.addLine(target + "..target");
//                    telemetry.addLine(((dif / 3000.0) + 0.05) + "..power");
//                    telemetry.update();
//                    dif = (target - Math.abs(motorSlide.getCurrentPosition()));
//                    motorSlide.setPower(((dif / 3000.0) + 0.05));
//                } else {
//                    motorSlide.setPower(0.05);
//                    inSlideLoop = false;
//                }
//            }
//            double leftY = -gamepad1.left_stick_y;
//            motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
//
//            while (true) {
//                motorSlide.setPower(leftY/3);
//            }
        }

    }

//    public void cali() {
//
//        // Set up the parameters with which we will use our IMU. Note that integration
//        // algorithm here just reports accelerations to the logcat log; it doesn't actually
//        // provide positional information.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//        }
//
//        imuIsCali = true;
//
//    }

//    public double getCurrentZ() {
//        if (imuIsCali == false) {
//            cali();
//        }
//
//        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//        double currentZ = currentAngle.firstAngle;
//        return currentZ;
//    }

//    public double getCumulativeZ() {
//        double currentHeading = getCurrentZ();
//        double deltaHeading = currentHeading - prevHead;
//
//        if (deltaHeading <= -180) {
//            deltaHeading += 360;
//        } else if (deltaHeading >= 180) {
//            deltaHeading -= 360;
//        }
//
//        inteHead += deltaHeading;
//        prevHead = currentHeading;
//
//        return inteHead;
//    }

    public void GoToHeight(int Clicks) {
        int target = (Clicks);
        int dif = (target - Math.abs(motorSlide.getCurrentPosition()));
        double aggressiveness = 3000;
        double holdingPower = 0;
        if (dif < 0) {
            aggressiveness = 3000;
            holdingPower = 0;
        } if (dif > 0) {
            aggressiveness = 2000;
            holdingPower = 0.05;
        }
        motorSlide.setPower((dif / aggressiveness));

        while (Math.abs(dif) >= 150) { // doesn't work when trying to go down
            telemetry.addLine(dif + "..difference");
            telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
            telemetry.addLine(target + "..target");
            telemetry.addLine(((dif / aggressiveness) + holdingPower) + "..power");
            telemetry.update();
            dif = (target - Math.abs(motorSlide.getCurrentPosition()));
            motorSlide.setPower(((dif / aggressiveness) + holdingPower));
        }
        motorSlide.setPower(holdingPower);
    }
}