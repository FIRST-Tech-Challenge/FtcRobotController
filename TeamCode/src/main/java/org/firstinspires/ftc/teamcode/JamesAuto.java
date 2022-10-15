package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveMethods;
import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name="JamesAuto", group="B")
public class
JamesAuto extends DriveMethods {
    boolean calibrated = false;
    double previousZ = 0;
    double integratedZ = 0;
    BNO055IMU imu;
    DcMotor motorLinearSlide;
    @Override
    public void runOpMode() {

        initMotorsBlue();


        motorLinearSlide = hardwareMap.get(DcMotor.class,"motorLS");
        motorLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

//        driveForDistance(1.25,0.5,Direction.FORWARD);
//        driveForDistance(0.15,0.4,Direction.RIGHT);
//        driveForDistance(3.5,0.7,Direction.LEFT);
//        driveForDistance(1.7,0.75,Direction.FORWARD);
//        driveForDistance(0.3,0.2,Direction.FORWARD);
        while (opModeIsActive()) {
            linearMotor(0.1234);
            telemetry.addLine("Current Angle Off: " + CumulativeZ());
            telemetry.update();
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

    public double currentZ(){
        if (!calibrated) {
            CalibrateIMU();
        }
        Orientation CurrentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentz = CurrentAngle.firstAngle;
        return currentz;
    }

    public double CumulativeZ(){
        double currentZ = currentZ();
        double deltaZ = currentZ-previousZ;
        if (deltaZ<-180){
            deltaZ += 360;
        } else if (deltaZ>=180) {
            deltaZ -= 360;
        }
        integratedZ+=deltaZ;
        previousZ = currentZ();
        return integratedZ;
    }

    public void CalibrateIMU (){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        calibrated = true;
    }

    public void linearMotor(double meters) {
        if (meters<0.5) {
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double target_encoder = meters * 537.7 / 0.112;
            double dif = target_encoder - motorLinearSlide.getCurrentPosition();
            while (dif>12){
                dif = target_encoder - motorLinearSlide.getCurrentPosition();
                    motorLinearSlide.setPower(dif / 3000);
            }
        }
    }

    public void driveDirection(Direction direction, double power){
        switch (direction) {
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

    public void driveForTime(int seconds, double power, Direction direction){
        //Fl is 0
        //Bl is 1
        //Fr is 2
        //Br is 3
        driveDirection(direction,power);

        int mili = seconds*1000;
        sleep(mili);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    public void driveForDistance(double distance, double power, Direction direction) {

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double targetClicks = distance*clicksPerRotation*rotationsPerMeter;

        motorBL.setTargetPosition((int)(targetClicks));
        motorBR.setTargetPosition((int)(targetClicks));
        motorFR.setTargetPosition((int)(targetClicks));
        motorFL.setTargetPosition((int)(targetClicks));

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while (avg<=targetClicks){

            driveDirection(direction,power);

            avg = (Math.abs((motorFL.getCurrentPosition()))+ Math.abs(motorBL.getCurrentPosition())+ Math.abs(motorFR.getCurrentPosition())+ Math.abs(motorBR.getCurrentPosition()))/4;

        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);


    }
}
