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

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontPosition = 0;
        leftBackPosition = 0;
        rightFrontPosition = 0;
        rightBackPosition = 0;


        waitForStart();



        angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        runtime.reset();


        drive(1000, 1000, 1000, 1000, 0.5, 100000, angels.firstAngle);
    }

        private void drive ( int leftFrontTarget,
        int leftBackTarget,
        int rightFrontTarget,
        int rightBackTarget, double speed, double ticks, float targetAngle){

            double leftFrontPower = speed;
            double leftBackPower = speed;
            double rightFrontPower = speed;
            double rightBackPower = speed;
//
//            rightFrontDrive.setPower(rightFrontPower);
//            leftFrontDrive.setPower(leftFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);
//
//            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + 3 );
//            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + 3);
//            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + 3);
//            rightBackDrive.setTargetPosition(rightBackDrive.getTargetPosition() + 3);




            while (leftFrontDrive.getCurrentPosition() < ticks && rightFrontDrive.getCurrentPosition() <
                    ticks && leftBackDrive.getCurrentPosition() < ticks && rightBackDrive.getCurrentPosition()
                    < ticks && opModeIsActive()) {
                angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addData("yaw:", " %f", angels.firstAngle);
                telemetry.addData("pitch", "%f", angels.secondAngle);
                telemetry.addData("role", "%f", angels.thirdAngle);
                telemetry.addData("angel:", "%f", angels.firstAngle);
                telemetry.addData("Encoder count: ","%7d",leftFrontDrive.getCurrentPosition());
                telemetry.addData("Encoder count: ","%7d",leftBackDrive.getCurrentPosition());
                telemetry.addData("Encoder count: ","%7d",rightFrontDrive.getCurrentPosition());
                telemetry.addData("Encoder count: ","%7d",rightBackDrive.getCurrentPosition());
                telemetry.update();


                if (angels.firstAngle < targetAngle || angels.firstAngle > targetAngle) {
                    double yaw = checkDirection();

                    leftFrontPower += yaw;
                    leftBackPower += yaw;
                    rightFrontPower -= yaw;
                    rightBackPower -= yaw;

//                    rightFrontDrive.setPower(rightFrontPower);
//                    leftFrontDrive.setPower(leftFrontPower);
//                    leftBackDrive.setPower(leftBackPower);
//                    rightBackDrive.setPower(rightBackPower);


                    telemetry.addData("angle: ", "%f", angels.firstAngle);
                }



                leftFrontPosition += leftFrontTarget;
                leftBackPosition += leftBackTarget;
                rightFrontPosition += rightFrontTarget;
                rightBackPosition += rightBackTarget;

//                leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + 3 );
//                leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + 3);
//                rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + 3);
//                rightBackDrive.setTargetPosition(rightBackDrive.getTargetPosition() + 3);
//
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            }
            }




    private float Angle() {

        float targetAngle = angels.firstAngle;
        float CurrentAngle;
        Orientation angels2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        CurrentAngle = angels2.firstAngle;

        float deltaAngle = targetAngle - CurrentAngle;

        return deltaAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10,yaw;

        angle = Angle();

        if (angle == 0) {
            correction = 0;
        }// no adjustment.
        else {
            correction = -angle;        // reverse sign of angle for correction.
        }
        correction = correction * gain;

        yaw = correction;



        return correction;
    }


   }










