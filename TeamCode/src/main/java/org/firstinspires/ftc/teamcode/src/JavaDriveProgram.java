package org.firstinspires.ftc.teamcode.src;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robotAttachments.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robotAttachments.Grabber;
import org.firstinspires.ftc.teamcode.robotAttachments.LinearSlide;
import org.firstinspires.ftc.teamcode.robotAttachments.TeleopDriveTrain;




@TeleOp(name = "2022 Drive Program")
public class JavaDriveProgram extends LinearOpMode {

    private BNO055IMU imu;

    private TeleopDriveTrain driveTrain;

    private Grabber grabber;
    private LinearSlide slide;
    private CarouselSpinner spinner;

    double Yaw_value;

    boolean grabberIsOpen;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    //Override
    public void runOpMode() throws InterruptedException {

        double DrivePowerMult = 1;
        boolean xDepressed = true;
        boolean yDepressed = true;
        boolean aDepressed = true;
        boolean bDepressed = true;

        /**
         * Drive-train, odometry and IMU initialization
         */
        driveTrain = new TeleopDriveTrain(hardwareMap,"back_left","back_right","front_left","front_right");

        //Init IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        grabber = new Grabber(hardwareMap, "freight grabber");
        grabber.open();
        grabberIsOpen = true;

        slide = new LinearSlide(hardwareMap, "linear slide arm");

        spinner = new CarouselSpinner(hardwareMap, "carousel wheel");


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                driveTrain.setPowerFromGamepad(gamepad1);

                //Handles Linear Slide Control
                slide.setMotorPower(0.75*gamepad2.left_stick_y);

                //Toggles Attachment A Button for Grabber, Needs some work. Sometimes doesn't toggle
                {
                    if (gamepad2.a==false) {
                        aDepressed = true;
                    }
                    if (gamepad2.a && grabberIsOpen && aDepressed) {
                        aDepressed = false;
                        grabberIsOpen = false;
                        grabber.close();
                    }
                    if (gamepad2.a && !grabberIsOpen && aDepressed) {
                        aDepressed = false;
                        grabberIsOpen = true;
                        grabber.open();
                    }
                }


                //Gamepad 2 B Red Duck
                {
                    if (gamepad2.x == false) {
                        bDepressed = true;
                    }
                    if (gamepad2.x && bDepressed) {
                        driveTrain.stopAll();
                        spinner.spinOffRedDuck();
                        bDepressed = false;
                    }
                }

                //Gamepad2 X BlueDuck
                {
                    if (gamepad2.b == false) {
                        bDepressed = true;
                    }
                    if (gamepad2.b && bDepressed) {
                        driveTrain.stopAll();
                        spinner.spinOffBlueDuck();
                        bDepressed = false;
                    }
                }

                if (gamepad1.b) {
                    driveTrain.setDrivePowerMult(0.3);
                }
                if (gamepad1.x) {
                    driveTrain.setDrivePowerMult(1);
                }
                if (gamepad1.a) {
                    driveTrain.setDrivePowerMult(0.6);
                }


            }

        }
    }


    private static double boundNumber(double num) {
        if (num > 1) {
            num = 1;
        }
        if (num < -1) {
            num = -1;
        }
        return num;
    }


    double getImu() {
        double returnVal = 0;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }


    private void InitIMU() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Calibrate automatically
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        //telemetry.addData("IMU initalized", "Press start to continue...");
        Yaw_value = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //telemetry.addData("Yaw_value", Yaw_value);
        //telemetry.update();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            while (true) {
                telemetry.addData("Exception:", e.toString());
                telemetry.update();
            }
        }
    }


    private double getVoltage() {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            return sensor.getVoltage();
        }

        return 0;

    }






}