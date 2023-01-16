package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Concept Datalog", group = "Datalogging")
public class ConceptDatalogger extends LinearOpMode
{
    Datalog datalog;
    BNO055IMU imu;
    VoltageSensor battery;
    Acceleration acceleration;

    MotorEx frontLeft, frontRight, rearLeft, rearRight;
    Motor.Encoder frontLeftE, frontRightE, rearLeftE, rearRightE;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        rearLeft = new MotorEx(hardwareMap, "rearLeft");
        rearRight = new MotorEx(hardwareMap, "rearRight");

        frontLeftE = frontLeft.encoder;
        frontRightE = frontRight.encoder;
        rearLeftE = rearLeft.encoder;
        rearRightE = rearRight.encoder;

        // Initialize the datalog
        datalog = new Datalog("accel");

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        for (int i = 0; opModeIsActive(); i++)
        {
            acceleration = imu.getLinearAcceleration();
            datalog.accelX.set(acceleration.xAccel);
            datalog.accelY.set(acceleration.yAccel);
            datalog.accelZ.set(acceleration.zAccel);
//            datalog.frontLeftTicks.set(frontLeftE.getPosition());
//            datalog.frontRightTicks.set(frontRightE.getPosition());
//            datalog.rearLeftTicks.set(rearLeftE.getPosition());
//            datalog.rearRightTicks.set(rearRightE.getPosition());

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine() ;

            telemetry.addLine();
            telemetry.addData("Accel X: ", datalog.accelX);
            telemetry.addData("Accel Y: ", datalog.accelY);
            telemetry.addData("Accel Z: ", datalog.accelZ);
//            telemetry.addData("Front Left: ", datalog.frontLeftTicks);
//            telemetry.addData("Front Right: ", datalog.frontRightTicks);
//            telemetry.addData("Rear Left: ", datalog.rearLeftTicks);
//            telemetry.addData("Rear Right: ", datalog.rearRightTicks);

            telemetry.update();

            sleep(20);
        }

        /*
         * The datalog is automatically closed and flushed to disk after
         * the OpMode ends - no need to do that manually :')
         */
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField accelX          = new Datalogger.GenericField("accelX");
        public Datalogger.GenericField accelY        = new Datalogger.GenericField("accelY");
        public Datalogger.GenericField accelZ         = new Datalogger.GenericField("accelZ");
        public Datalogger.GenericField frontLeftTicks         = new Datalogger.GenericField("frontLeftTicks");
        public Datalogger.GenericField frontRightTicks         = new Datalogger.GenericField("frontRightTicks");
        public Datalogger.GenericField rearLeftTicks         = new Datalogger.GenericField("rearLeftTicks");
        public Datalogger.GenericField rearRightTicks         = new Datalogger.GenericField("rearRightTicks");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            accelX,
                            accelY,
                            accelZ,
                            frontLeftTicks,
                            frontRightTicks,
                            rearLeftTicks,
                            rearRightTicks
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
