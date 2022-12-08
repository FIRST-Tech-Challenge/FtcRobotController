package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous

public class BlueLeft extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    OpenCvWebcam webcam;
    private DcMotor Spin;
    private DcMotor Crane;
    private CRServo Left;


    BNO055IMU imu;
    Orientation angles;

    public void runOpMode() throws InterruptedException {
        initGyro();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        Left = hardwareMap.get(CRServo.class, "Lefts");
        Spin = hardwareMap.get(DcMotor.class, "Spin");
        Crane = hardwareMap.get(DcMotor.class, "Crane");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();//possubly the problem
        if (opModeIsActive()) {



            move(1, 1900);
            gyroTurning(90);
            move(1, 750);

            Left.setPower(-1);
            craneinput();
            sleep(1000);
            Left.setPower(0);

            move(1,-500);
            gyroTurning(180);
            spin(100, -1600);
            sleep(1000);
            move(1, 500);

            /*

            crane(-1500);
            Left.setPower(-1);
            crane(-1600);
            move(1,-200);
            crane(0);
            */


        }
    }


    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sleep(250);
    }

    public boolean gyroTurning(double targetAngle) {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean foundAngle;
        foundAngle = false;
        while (!foundAngle) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            telemetry.addData("Angle", currentAngle);
            telemetry.addData("targetangle", targetAngle);
            telemetry.update();
            if (angles.firstAngle >= targetAngle - 0.1 && angles.firstAngle <= targetAngle + 0.1) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                foundAngle = true;
                sleep(1000);
                break;

            } else if (angles.firstAngle >= targetAngle + 0.5) {
                if (angles.firstAngle <= targetAngle - 5) {
                    frontLeft.setPower(0.2);
                    frontRight.setPower(-0.2);
                    backLeft.setPower(0.2);
                    backRight.setPower(-0.2);
                    foundAngle = false;
                } else {
                    frontLeft.setPower(-0.2);
                    frontRight.setPower(0.2);
                    backLeft.setPower(-0.2);
                    backRight.setPower(0.2);
                    foundAngle = false;
                }
            } else if (angles.firstAngle <= targetAngle - 0.5) {
                if (angles.firstAngle >= targetAngle + 5) {
                    frontLeft.setPower(-0.2);
                    frontRight.setPower(0.2);
                    backLeft.setPower(-0.2);
                    backRight.setPower(0.2);
                    foundAngle = false;
                } else {
                    frontLeft.setPower(.2);
                    frontRight.setPower(-.2);
                    backLeft.setPower(.2);
                    backRight.setPower(-.2);
                    foundAngle = false;
                }
            }
        }
        return foundAngle;
    }

    public void stopMotors() throws InterruptedException {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void move(double power, int position) {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition(-position);
        frontLeft.setTargetPosition(-position);
        backRight.setTargetPosition(-position);
        backLeft.setTargetPosition(-position);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        while (frontLeft.isBusy() && opModeIsActive()) {

        }

    }
    public void craneinput() {
        crane(-500);
        move(0.5, 350);
        //Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Crane.setTargetPosition(0);
        Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Crane.setPower(1);
        while (Crane.isBusy() && opModeIsActive()) {
        }
    }

    public void strafeLeft(double power, int time) throws InterruptedException {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);
        stopMotors();
    }

    public void strafeRight(double power, int time) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void intake(int direction, long time) throws InterruptedException {
        Left.setPower(direction * 1);
        sleep(time);
        Left.setPower(0);

    }

    /*public void moveandspin(double power, int moveposition, int spinposition) {
        move(power, moveposition);
        spin(spinposition);
        while (Spin.isBusy()) {

        }
    }*/

    public void spin(int SpinPosition,int CranePosition) {
        Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spin.setTargetPosition(SpinPosition);
        Crane.setTargetPosition(CranePosition);
        Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Spin.setPower(1);
        Crane.setPower(1);
        while (Spin.isBusy()&&opModeIsActive()) {

        }
    }

    public void crane(int position) {
        Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Crane.setTargetPosition(position);
        Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Crane.setPower(1);
        while (Crane.isBusy()&&opModeIsActive()) {

        }

    }
}

