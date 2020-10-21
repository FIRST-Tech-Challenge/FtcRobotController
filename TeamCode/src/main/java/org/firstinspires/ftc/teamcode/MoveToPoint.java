package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "DeadREckoning1", group = "DreamMachines")
public class MoveToPoint extends LinearOpMode {

    public DMHardware robot = new DMHardware(true);

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER_CM = 8.89;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.3;

    double currentPosX = 0;
    double currentPosY = 0;

    BNO055IMU imu;

    @Override
    public void runOpMode() {

        robot.initTeleOpIMU(hardwareMap);
        robot.timer.reset();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        waitForStart();

        // Calling methods...
        moveToPoint(90, 180, 120);


    }

    public void encoderDrive(double speed,double rightCM, double leftCM, double timeout) {

        int newLeftTarget;
        int newRightTarget;

        if(opModeIsActive()) {
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(-1 * (leftCM * COUNTS_PER_CM));
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(-1 * (rightCM * COUNTS_PER_CM));

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.timer.reset();

            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);



            while (opModeIsActive() && robot.timer.seconds() < timeout && robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
                telemetry.addData("Path", "Going to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Currently at %7d :&7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());

                telemetry.update();
            }

            robot.setPowerOfAllMotorsTo(0);

            // Reset all motors
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }public void turnWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
            double error = targetAngle - getHeading();
            double sum = error;
            double previousError = error;
            double correction = 0;
            double slope = 0;
            robot.timer.reset();
            while (Math.abs(error) > threshold && opModeIsActive()) {
                telemetry.addData("error", error);
                telemetry.update();
                error = targetAngle - getHeading();
                slope = (error - previousError) / (double) robot.timer.time();
                robot.timer.reset();
                sum += error;
                correction = kp * error + ki * sum + kd * slope;


                robot.leftMotor.setPower(correction);
                robot.rightMotor.setPower(-1 * correction );
            }
        }
    public void moveToPoint(double xCoord, double yCoord, int heading) {
        double angle = (Math.tan((currentPosX - xCoord) / (currentPosY - yCoord)));
        angle = Math.toDegrees(angle);
        double hypotenuse = Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2));

        //Turn
        turnWithPID(angle, 0.085, 0, 0.0001, 1);

        robot.setPowerOfAllMotorsTo(0);

        //Move
        encoderDrive(0.4, hypotenuse, hypotenuse, 100);

        robot.setPowerOfAllMotorsTo(0);

        //Turn to heading
        turnWithPID(heading, 0.085, 0, 0.0001, 1);
        currentPosX = xCoord;
        currentPosY = yCoord;

        robot.setPowerOfAllMotorsTo(0);




    }
}
