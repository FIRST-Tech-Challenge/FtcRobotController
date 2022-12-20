package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IMU drive by encoders")
public class driveByIMUAndEncoder extends BasicOpMode_Linear {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    BNO055IMU imu;
    Orientation angels;

    private int leftFrontPosition;
    private int leftBackPosition;
    private int rightFrontPosition;
    private int rightBackPosition;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontPosition = 0;
        leftBackPosition = 0;
        rightFrontPosition = 0;
        rightBackPosition = 0;


        waitForStart();

        angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        runtime.reset();


        drive(1000, 1000, 1000, 1000, 0.5, 100000, angels.firstAngle);
    }


    private void drive(int leftFrontTarget,
                       int leftBackTarget,
                       int rightFrontTarget,
                       int rightBackTarget, double speed, double ticks, float targetAngle) {

        while (leftFrontDrive.getCurrentPosition() < ticks && rightFrontDrive.getCurrentPosition() <
                ticks && leftBackDrive.getCurrentPosition() < ticks && rightBackDrive.getCurrentPosition()
                < ticks) {
            angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("yaw:", " %f", angels.firstAngle);
            telemetry.addData("pitch", "%f", angels.secondAngle);
            telemetry.addData("role", "%f", angels.thirdAngle);
            telemetry.addData("angel:", "%f", angels.firstAngle);
            telemetry.update();



            if(angels.firstAngle < targetAngle){
                angels.firstAngle += angels.firstAngle;
                telemetry.addData("angle: ","%f",angels.firstAngle);
            }
            if(angels.firstAngle > targetAngle){
                angels.firstAngle -= angels.firstAngle;
                telemetry.addData("angle: ","%f",angels.firstAngle);
            }
            else {
                rightFrontDrive.setPower(1.0);
                leftFrontDrive.setPower(1.0);
                leftBackDrive.setPower(1.0);
                rightBackDrive.setPower(1.0);
            }

            leftFrontPosition += leftFrontTarget;
            leftBackPosition += leftBackTarget;
            rightFrontPosition += rightFrontTarget;
            rightBackPosition += rightBackTarget;

            leftFrontDrive.setTargetPosition(leftFrontPosition);
            leftBackDrive.setTargetPosition(leftBackPosition);
            rightFrontDrive.setTargetPosition(rightFrontPosition);
            rightBackDrive.setTargetPosition(rightBackPosition);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            resetRuntime();


        }


        }
    }





