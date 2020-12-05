package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import java.io.File;

@Autonomous
public class Calibration extends LinearOpMode{

    DcMotor frontRight, frontLeft, backRight, backLeft;
    DcMotor leftEncoder, rightEncoder, middleEncoder;

    //Inertial Measurement Unit i think it measures movement but idk
    BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();

    //speed
    static final double calibrationSpeed = 0.5;

    //figure this out later
    //for wheel if diameter 100 m divide by 25.4 to keep inches
    static final double TICK_PER_REV = 0;
    static final double WHEEL_DIAMETER = 0;
    static final double GEAR_RATIO =1; // only for gears but if not just set as 1 because 1:1

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO/TICK_PER_REV;

    //creates text files on phone that are used in odometry
    File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    @Override
    public void runOpMode() throws InterruptedException {
        //initalizing variables
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        //if we need to reverse we would put negative and NOT REVERSE ENCODER because wheel might reverse
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("middleEncoder");

       resetOdometryEncoders();

       //imu is built into the rev hub and the imu get acceleration and can
        //be used instead of a gyro(which i think does the same thing)
        //can use to drive in a straight line i think
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // looks like BNO055IMU.Paramters is a object or smth
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //i think initilizing differnt variables in the object
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        telemetry.addData("Status","Ready!");
        telemetry.update();

        waitForStart();

        //might be different angle
        //should probaly look more into this but yeah
        while(imu.getAngularOrientation().firstAngle < 90 && opModeIsActive())
        {
            frontRight.setPower(-calibrationSpeed);
            backRight.setPower(-calibrationSpeed);
            frontLeft.setPower(calibrationSpeed);
            backLeft.setPower(calibrationSpeed);

            if(imu.getAngularOrientation().firstAngle<60)
            {
                frontRight.setPower(-calibrationSpeed);
                backRight.setPower(-calibrationSpeed);
                frontLeft.setPower(calibrationSpeed);
                backLeft.setPower(calibrationSpeed);
            }else{
                //slow down
                frontRight.setPower(-calibrationSpeed/2);
                backRight.setPower(-calibrationSpeed/2);
                frontLeft.setPower(calibrationSpeed/2);
                backLeft.setPower(calibrationSpeed/2);
            }
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        timer.reset();
        while(timer.seconds()<1 & opModeIsActive())
        {
            //does nothing
        }

        double angle = imu.getAngularOrientation().firstAngle;
        //absoulte value of subtracting the absolyte value of encoders basoclly differece between right and left
        double encoderDifference = Math.abs(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()));
        double sideEncoderTickOffset = encoderDifference/angle;
        double sideWheelSeperation = (180*sideEncoderTickOffset)/(TICKS_PER_INCH * Math.PI);
        double middleTickOffset = middleEncoder.getCurrentPosition() / Math.toRadians(imu.getAngularOrientation().firstAngle);

        //puts stuff in file
        ReadWriteFile.writeFile(sideWheelSeparationFile, String.valueOf(sideWheelSeperation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleTickOffset));
    }

    private void resetOdometryEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
