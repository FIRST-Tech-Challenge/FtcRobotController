package org.firstinspires.ftc.teamcode.IMU;

import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;
import static java.lang.Thread.sleep;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.controller.MechanicalDriveBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import java.util.Locale;



/** removed distance sensor part of code now for rotate mainly - Sam**/
public class IMUControl
{
    private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

    private MechanicalDriveBase mechanicalDriveBase;

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, strafeCorrection,  rotation;
    PIDController pidRotate, pidDrive, pidStrafe;
    Orientation angles;
    Acceleration gravity;

    BNO055IMU.Parameters myIMUparameters;

    public IMUControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
//        teamDetection = new TeamDetection(hardwareMap);
//        mechanicalDriveBase = new MechanicalDriveBase(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .0000/*3*/, 0);

        // Set PID pro
        // portional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        pidStrafe = new PIDController(.05, 0, 0);

        composeTelemetry(telemetry);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated())
        {
            try
            {
                sleep(50);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }

//        resetAngle();

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
    }

    void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // called when init button is  pressed.
//    public void Run(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException
//    {
//        IMUControl(hardwareMap, telemetry);
//
//        // wait for start button
//
//        telemetry.addData("Mode", "running");
//        telemetry.update();
//
//        sleep(1000);
//
//        //TODO: Do we do this just once?  Can the same settings be used for strafe?
//        // Set up parameters for driving in a straight line.
//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(-power, power);
//        pidDrive.setInputRange(-90, 90);
//        pidDrive.enable();
//
///*        //TODO: Same settings as drive????
//        pidStrafe.setSetpoint(0);
//        pidStrafe.setOutputRange(-power, power);
//        pidStrafe.setInputRange(-90, 90);
//        pidStrafe.enable(); */
//
//        //Drive forward 18 inches
//        driveStraight(18 * ticksPerInch, 1, 0.1, telemetry);
//        rotate(180, 0.3);
//        sleep(1500);
//    }

    //Set target then multiply by one with negative if you want to go backwards no negative input
    private void driveStraight(double target, int forward, double speed, Telemetry telemetry)
    {
        speed *= forward;

        //Fetch the odometry pod wheel location.
        leftFrontPos = mechanicalDriveBase.lf.getCurrentPosition();
        if (forward == 1)
        {
            //Add the target distance to the current location
            leftFrontPos += target;

            //Drive from current position to target position
            while (mechanicalDriveBase.lf.getCurrentPosition() <= leftFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                correction = pidDrive.performPID(getAngle());
//                correction *= 0.100;
                //Pass the correction value into the turn param.  No idea what kind of range will
                //be on this value.  Should be small value like 0.1 or less I would hope.
                mechanicalDriveBase.driveMotors(speed, correction, 0, 1);

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.addData("5 lf pos", mechanicalDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            leftFrontPos -= target;
            while (mechanicalDriveBase.lf.getCurrentPosition() >= leftFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                correction = pidDrive.performPID(getAngle());

                mechanicalDriveBase.driveMotors(speed, correction, 0, 1);

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.addData("5 lf pos", mechanicalDriveBase.lf.getCurrentPosition());
                telemetry.update();
            }
        }
        mechanicalDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, - is left + is right
     */
    public double rotate(int degrees, double power)
    {
        degrees *= -1;
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0.5, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0)
            {
                mechanicalDriveBase.driveMotors(0, power, 0, 1);

                try
                {
                    sleep(100);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                mechanicalDriveBase.driveMotors(0, power, 0, 1);

            }
            while (!pidRotate.onTarget());

        }
        else
        {
            // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //leftMotor.setPower(-power);
                //rightMotor.setPower(power);
            }
            while (!pidRotate.onTarget());
        }

        mechanicalDriveBase.driveMotors(0, 0, 0, 0);

        rotation = getAngle();

        // wait for rotation to stop.
        try
        {
            sleep(500);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        // reset angle tracking on new heading.
        resetAngle();

        //TODO: return the number of degrees it did turn.  (in case distance preempted turn)
        return 0.0;
    }

    //Set target then multiply by one with negative if you want to go left currently set right no negative input
    private void strafe(double target, int right, double speed, Telemetry telemetry)
    {
        speed *= right;
        if (right == 1)
        {
            rightFrontPos -= target;
            while (mechanicalDriveBase.rf.getCurrentPosition() >= rightFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                strafeCorrection = pidStrafe.performPID(getAngle());

                mechanicalDriveBase.driveMotors(0, strafeCorrection, speed, 1);
                telemetry.addData("", mechanicalDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            rightFrontPos += target;
            while (mechanicalDriveBase.rf.getCurrentPosition() <= rightFrontPos)
            {
                // Use PID with imu input to drive in a straight line.
                strafeCorrection = pidStrafe.performPID(getAngle());

                mechanicalDriveBase.driveMotors(0, strafeCorrection, speed, 1);
                telemetry.addData("", mechanicalDriveBase.rf.getCurrentPosition());
                telemetry.update();
            }
        }
        mechanicalDriveBase.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}