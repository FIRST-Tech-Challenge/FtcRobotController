package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Move 10 centimeters", group="Auton Test Suite")
public class TravelXDistance extends LinearOpMode
{
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double leftFrontMotorPos, leftFrontDistanceTraveled = 0, deltaLeftFrontMotorPos = 0, previousLeftFrontMotorPos, leftFrontPower;
    public double rightFrontMotorPos, rightFrontDistanceTraveled = 0, deltaRightFrontMotorPos = 0, previousRightFrontMotorPos, rightFrontPower;
    public double leftBackMotorPos, leftBackDistanceTraveled = 0, deltaLeftBackMotorPos = 0, previousLeftBackMotorPos, leftBackPower;
    public double rightBackMotorPos, rightBackDistanceTraveled = 0, deltaRightBackMotorPos = 0, previousRightBackMotorPos, rightBackPower;
    double positiveDistanceTraveled, negativeDistanceTraveled;
    double distancePerTick = (2 * Math.PI * 48) / 537.6;
    int instruction = 1;
    double globalAngle = 0;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        waitForStart();

        runtime.reset();

        while (opModeIsActive())
        {
            // Fill this section with variables when your shit doesn't work
            telemetry.addData("1 runtime", runtime.milliseconds());
            telemetry.addData("2 distance traveled per cycle", deltaLeftFrontMotorPos);
            telemetry.addData("3 left front position", leftFrontDistanceTraveled);
            telemetry.update();

            // Call functions to actually move the thing
            move(300, 300);
        }
    }

    void move(double posTarget, double negTarget)
    {
        // Update the positions of each individual motor
        leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
        deltaLeftFrontMotorPos = distancePerTick * (leftFrontMotorPos - previousLeftFrontMotorPos);
        leftFrontDistanceTraveled += deltaLeftFrontMotorPos;
        previousLeftFrontMotorPos = leftFrontMotorPos;

        rightFrontMotorPos = rightFrontMotor.getCurrentPosition();
        deltaRightFrontMotorPos = distancePerTick * (rightFrontMotorPos - previousRightFrontMotorPos);
        rightFrontDistanceTraveled += deltaRightFrontMotorPos;
        previousRightFrontMotorPos = rightFrontMotorPos;

        leftBackMotorPos = leftBackMotor.getCurrentPosition();
        deltaLeftBackMotorPos = distancePerTick * (leftBackMotorPos - previousLeftBackMotorPos);
        leftBackDistanceTraveled += deltaLeftBackMotorPos;
        previousLeftBackMotorPos = leftBackMotorPos;

        rightBackMotorPos = rightBackMotor.getCurrentPosition();
        deltaRightBackMotorPos = distancePerTick * (rightBackMotorPos - previousRightBackMotorPos);
        rightBackDistanceTraveled += deltaRightBackMotorPos;
        previousRightBackMotorPos = rightBackMotorPos;

        // Get the distance between the target and the position
        double leftFrontError = posTarget - leftFrontDistanceTraveled;
        double rightFrontError = negTarget - rightFrontDistanceTraveled;
        double leftBackError = negTarget - leftBackDistanceTraveled;
        double rightBackError = posTarget - rightBackDistanceTraveled;

        // Calculate the motor powers
        if (positiveDistanceTraveled <= (posTarget + 15) || positiveDistanceTraveled >= (posTarget + 25)) {
            leftFrontPower = 0.034 * leftFrontError;
            rightBackPower = 0.034 * rightBackError;
        }
        if (negativeDistanceTraveled <= (negTarget + 15) || negativeDistanceTraveled >= (negTarget + 25)) {
            rightFrontPower = 0.034 * rightFrontError;
            leftBackPower = 0.034 * leftBackError;
        }
        // Stop if we reach the target position
        else
        {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            instruction++;
        }

        // Limit motor power so we don't lose accuracy
        if (leftFrontPower > 0.35) { leftFrontPower = 0.35; }
        if (rightFrontPower > 0.35) { rightFrontPower = 0.35; }
        if (leftBackPower > 0.35) { leftBackPower = 0.35; }
        if (rightBackPower > 0.35) { rightBackPower = 0.35; }

        // Apply motor power
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    void turn(double degrees)
    {
        // Get the current orientation of the bot
        double angle = getOrientation();

        degrees -= 13;

        // Make sure we're not at the target. If not, move. If so, stop.
        if (angle < degrees) {
            leftFrontPower = (degrees - angle);
            rightFrontPower = -(degrees - angle);
            leftBackPower = (degrees - angle);
            rightBackPower = -(degrees - angle);
        } else
        {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            instruction++;
        }

        // Limit motor power so we don't lose accuracy
        if (leftFrontPower > 0.35) { leftFrontPower = 0.35; }
        if (rightFrontPower < -0.35) { rightFrontPower = -0.35; }
        if (leftBackPower > 0.35) { leftBackPower = 0.35; }
        if (rightBackPower < -0.35) { rightBackPower = -0.35; }

        // Apply motor power
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    private double getOrientation()
    {
        // Get the orientation from the gyroscope
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // The number we get from the gyro has no limit. We can technically use it like this but
        // that would be messy.
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        // This value is now the orientation of the bot represented by a degree between -180 and 180
        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}

