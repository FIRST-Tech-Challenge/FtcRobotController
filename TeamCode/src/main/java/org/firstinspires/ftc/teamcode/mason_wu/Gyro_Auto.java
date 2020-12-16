package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@Autonomous(name="Mechanum AutoOp", group="Iterative Opmode")
public class Gyro_Auto extends LinearOpMode
{
    // Declare OpMode members.
    private DcMotor shooter = null;
    private Servo spanker = null;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {

        // Initialize the hardware variables.
        spanker = hardwareMap.get(Servo.class,"spanker");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        spanker.setPosition(0.4);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());

        // composeTelemetry();

        waitForStart();

        if (opModeIsActive()) {
            ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            while(t.seconds()<=15){
                driveStraight(5,0.3);
            }
        }

    }

    double aquireHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempDeg = heading % 360;
        if(tempDeg >= 180){
            tempDeg -= 360;
        }else if(tempDeg < -180){
            tempDeg += 360;
        }
        telemetry.addData("Heading", aquireHeading());
        telemetry.update();
        sleep(50);
        return tempDeg;
    }
    void driveStraight (double range, double power) throws InterruptedException{
        double currentAngle = aquireHeading();
        double LF_power = power;
        double LB_power = power;
        double RF_power = power;
        double RB_power = power;
        if(currentAngle<(-1*range)){
            RF_power += 0.1;
            RB_power += 0.1;
            LF_power -= 0.1;
            LB_power -= 0.1;
        }
        else if(currentAngle>(range)){
            RF_power -= 0.1;
            RB_power -= 0.1;
            LF_power += 0.1;
            LB_power += 0.1;
        }
        RF_power = Range.clip(RF_power,-1,1);
        RB_power = Range.clip(RB_power,-1,1);
        LF_power = Range.clip(LF_power,-1,1);
        LB_power = Range.clip(LB_power,-1,1);
        LF.setPower(LF_power);
        RF.setPower(RF_power);
        LB.setPower(LB_power);
        RB.setPower(RB_power);
        telemetry.addData("RF_power", RF_power);
        telemetry.addData("LF_power", LF_power);
        telemetry.addData("RB_power", RB_power);
        telemetry.addData("LB_power", LB_power);
        telemetry.update();
    }

    // void composeTelemetry() {

    //        // At the beginning of each telemetry update, grab a bunch of data
    //        // from the IMU that we will then display in separate lines.
    //        telemetry.addAction(new Runnable() { @Override public void run()
    //                {
    //                // Acquiring the angles is relatively expensive; we don't want
    //                // to do that in each of the three items that need that info, as that's
    //                // three times the necessary expense.
    //                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //                gravity  = imu.getGravity();
    //                }
    //            });

    //        telemetry.addLine()
    //            .addData("status", new Func<String>() {
    //                @Override public String value() {
    //                    return imu.getSystemStatus().toShortString();
    //                    }
    //                })
    //            .addData("calib", new Func<String>() {
    //                @Override public String value() {
    //                    return imu.getCalibrationStatus().toString();
    //                    }
    //                });

    //        telemetry.addLine()
    //            .addData("heading", new Func<String>() {
    //                @Override public String value() {
    //                    return formatAngle(angles.angleUnit, angles.firstAngle);
    //                    }
    //                })
    //            .addData("roll", new Func<String>() {
    //                @Override public String value() {
    //                    return formatAngle(angles.angleUnit, angles.secondAngle);
    //                    }
    //                })
    //            .addData("pitch", new Func<String>() {
    //                @Override public String value() {
    //                    return formatAngle(angles.angleUnit, angles.thirdAngle);
    //                    }
    //                });

    //        telemetry.addLine()
    //            .addData("grvty", new Func<String>() {
    //                @Override public String value() {
    //                    return gravity.toString();
    //                    }
    //                })
    //            .addData("mag", new Func<String>() {
    //                @Override public String value() {
    //                    return String.format(Locale.getDefault(), "%.3f",
    //                            Math.sqrt(gravity.xAccel*gravity.xAccel
    //                                    + gravity.yAccel*gravity.yAccel
    //                                    + gravity.zAccel*gravity.zAccel));
    //                    }
    //                });
    //    }

    //    String formatAngle(AngleUnit angleUnit, double angle) {
    //        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    //    }

    //    String formatDegrees(double degrees){
    //        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    //    }
}