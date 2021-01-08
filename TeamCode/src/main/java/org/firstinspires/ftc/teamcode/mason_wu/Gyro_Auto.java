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

@Autonomous(name="Gyro Autonomous", group="4100")
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
            //drive straight with a power of 0.4 for 5000 milliseconds, allowing a margin of error of 1 degree
            driveStraight(1.0,0.4,5000);
            sleep(500);
            //turn to 120 degrees with a power of 0.4
            turnToAngle(120,1.0,0.4);
            sleep(500);
            //drive straight with power of 0.4 for 2500 milliseconds, allowing a margin of error of 1 degree
            driveStraight(1.0,0.4,2500);
            sleep(500);
            //turn 90 degrees in counterclockwise direction with a power of 0.4
            turnAtAngle(false,90.0,1.0,0.4);
            sleep(500);
            //drive straight with a power of 0.4 for 4330 milliseconds, allowing a margin of error of 1 degree
            driveStraight(1.0,0.4,4330);
            sleep(500);
        }
    }

    void stopMotion(){
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
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
        telemetry.addData("Heading", tempDeg);
        telemetry.update();
        sleep(20);
        return tempDeg;
    }
    void driveStraight (double margin, double power, double timeInterval) throws InterruptedException{
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while(driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = power;
            LB_power = power;
            RF_power = power;
            RB_power = power;
            if (tempAngle < currentAngle -1 * margin) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > currentAngle + (margin)) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }
    //make a turn that based on the current heading in a certain direction and angle
    void turnAtAngle (boolean isClockwise, double degree, double margin, double power){
        int angleFactor = -1;
        if(!isClockwise){
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = currentAngle + degree * angleFactor;
        if(targetAngle >= 180){
            targetAngle -= 360;
        }else if(targetAngle < -180){
            targetAngle += 360;
        }
        turnToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void turnToAngle (double targetAngle, double margin, double power){
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if(currentAngle - targetAngle > 0){
            if(currentAngle - targetAngle < 180){
                //cw
                angleFactor = -1;
            }
            //ccw
            angleFactor = 1;
        }
        else{
            if(targetAngle - currentAngle < 180){
                //ccw
                angleFactor = 1;
            }
            //cw
            angleFactor = -1;
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin)&&(tempAngle > targetAngle - margin))){
            tempAngle = aquireHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }
}