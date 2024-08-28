package org.firstinspires.ftc.teamcode.williamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotInitialize {

    // Initialization Phase

    // Create servo variables
    Servo pitch;
    Servo lClaw;
    Servo rClaw;

    // Create the empty normal motor variables
    DcMotorEx fleft;
    DcMotorEx bright;
    DcMotorEx fright;
    DcMotorEx bleft;


    // Create empty gyroscope variable
    BNO055IMU gyroScope;
    BNO055IMU.Parameters settings;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    LinearOpMode opMode;

    public RobotInitialize(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }


    public void initialize() {
        // map the motors to the hardware map
        fleft = opMode.hardwareMap.get(DcMotorEx.class, "fleft");
        bright = opMode.hardwareMap.get(DcMotorEx.class, "bright");
        fright = opMode.hardwareMap.get(DcMotorEx.class, "fright");
        bleft = opMode.hardwareMap.get(DcMotorEx.class, "bleft");

        bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fleft.setDirection(DcMotorSimple.Direction.REVERSE);

// map the servos to the hardware map
        pitch = opMode.hardwareMap.get(Servo.class, "pitch");
        lClaw = opMode.hardwareMap.get(Servo.class, "lClaw");
        rClaw = opMode.hardwareMap.get(Servo.class, "rClaw");

        // Resetting the encoders (distance measurement sensors)
        // and then start them again on program start

        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Gyroscope
        gyroScope = opMode.hardwareMap.get(BNO055IMU.class, "Gyroscope");
        settings = new BNO055IMU.Parameters();
        settings.mode = BNO055IMU.SensorMode.IMU;
        settings.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        settings.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        settings.loggingEnabled = false;
        gyroScope.initialize(settings);

        while (!gyroScope.isGyroCalibrated()) {
            //Wait
            opMode.telemetry.addLine("GYRO WAITING...");
            opMode.telemetry.update();
        }
    }

    public void makeSquare() {
        goStraight(500, 500);
        newTurnFunction(-90);
        goStraight(500, 500);
        newTurnFunction(-180);
        goStraight(500, 500);
        newTurnFunction(-270);
        goStraight(500, 500);
        newTurnFunction(-360);
    }

    private double getAngle() {
        Orientation angles = gyroScope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    // This function makes the robot travel a relativeDistance specified by the parameter relativeDistance
    // This parameter is measured in encoder ticks; the other parameter, velocity, is
    // a decimal value that determines how fast the robot will go
    public void goStraight (int distance, double velocity) {
        // Travel a certain relativeDistance based on the absolute value
        // of the average robot encoder reading
        // The relativeDistance is current position plus or minus the value that is being moved
        // RELATIVE DISTANCE MEASUREMENT IN USE
        int relativeDistance = distance + getAverageEncoderValue();
        // Go forwards or backwards
        while (opMode.opModeIsActive() && Math.abs(getAverageEncoderValue() - relativeDistance) >= 10) {
            if (getAverageEncoderValue() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                setMotorVelocity(Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getAverageEncoderValue());
                opMode.telemetry.update();
            } else if (getAverageEncoderValue() > relativeDistance) {
                // Backwards (- negative relativeDistance value)
                setMotorVelocity(-Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getAverageEncoderValue());
                opMode.telemetry.update();
            }
        }
        stopMotors();
    }

    // Makes the robot turn a certain number of degrees
    // The parameter degrees is the amount of degrees the robot turns


    // This is the old turn function with setPower instead of set velocity

    /*
    public void oldTurnFunction(int degrees) {
        // When turning left, counterclockwise is a positive gyro value
        // When turning right, clockwise is a negative gyro value
        // ABSOLUTE POSITIONING IN USE (will go to exact values)

        while (opModeIsActive() && Math.abs(degrees - getAngle()) >= 0.5) {
            telemetry.addData("Encoder turn:", fleft.getCurrentPosition());
            telemetry.addData("Gyroscope", getAngle());
            telemetry.update();

            if (degrees > getAngle()) {
                // Turning left (positive gyro value)
                fleft.setPower(-0.2);
                bleft.setPower(-0.2);
                fright.setPower(0.2);
                bright.setPower(0.2);
            }
            else if (degrees < getAngle()) {
                // Turning right (negative gyro value)
                fleft.setPower(0.2);
                bleft.setPower(0.2);
                fright.setPower(-0.2);
                bright.setPower(-0.2);
            }
        }
        stopMotors();
    }
     */


    // This is the new turn function that includes setVelocity
    public void newTurnFunction(int degrees) {
        // When turning left, counterclockwise is a positive gyro value
        // When turning right, clockwise is a negative gyro value
        // ABSOLUTE POSITIONING IN USE (will go to exact values)

        while (opMode.opModeIsActive() && Math.abs(degrees - getAngle()) >= 0.2) {
            opMode.telemetry.addData("Encoder turn:", fleft.getCurrentPosition());
            opMode.telemetry.addData("Gyroscope", getAngle());
            opMode.telemetry.update();

            if (degrees > getAngle()) {
                // Turning left (positive gyro value)
                fleft.setVelocity(-500);
                bleft.setVelocity(-500);
                fright.setVelocity(500);
                bright.setVelocity(500);
            }
            else if (degrees < getAngle()) {
                // Turning right (negative gyro value)
                fleft.setVelocity(500);
                bleft.setVelocity(500);
                fright.setVelocity(-500);
                bright.setVelocity(-500);
            }
        }
        stopMotors();
    }


    // Calculates the average encoder value
    // It takes the left and right motor encoder locations, then averages them
    public int getAverageEncoderValue() {
        return ((getLeftSideEncoderValues() + getRightSideEncoderValues()) / 2);
    }

    // Calculates the average left side encoder values
    // It takes the left side encoder location of the left side motors and averages them
    public int getLeftSideEncoderValues() {
        return ((fleft.getCurrentPosition() + bleft.getCurrentPosition())/2);
    }

    // Calculates the average right side encoder values
    // It takes the right side encoder location of the right side motors and averages them
    public int getRightSideEncoderValues() {
        return ((fright.getCurrentPosition() + bright.getCurrentPosition())/2);
    }

    // Sets the motor power, a decimal (double) between -1 and 1
    // parameter power is of type double (a decimal) and stores the motor power
    // to set the power of the motors
    // This function does not return anything, it has (void)

    // This is the old method of controlling the motor speed

    /*
    public void setMotorPower(double power) {
        fleft.setPower(power);
        fright.setPower(power);
        bleft.setPower(power);
        bright.setPower(power);
    }
     */

    // This sets the movement of the motors to be constant
    public void setMotorVelocity(double velocity) {
        fleft.setVelocity(velocity);
        fright.setVelocity(velocity);
        bleft.setVelocity(velocity);
        bright.setVelocity(velocity);
    }

    public void stopMotors() {
        setMotorVelocity(0);
    }
}


