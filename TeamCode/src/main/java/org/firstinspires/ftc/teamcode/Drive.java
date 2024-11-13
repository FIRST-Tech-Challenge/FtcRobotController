package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drive {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;

    private BNO055IMU imu;

    public enum Direction {Forward, Backward, Right, Left};

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // 384.5 for 5203; 537.7 for 5202
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For calculating circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;


    private int targetEncoderValue;

    public Drive(HardwareMap hardwareMap, Telemetry telem) {

        telemetry = telem;

        initDriveMotors(hardwareMap);
    }

    private void initDriveMotors(HardwareMap hardwareMap)
    {
        // Initialize the drive motors

        // Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize encoders
        //leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set wheel direction. Important to reverse the direction of the left wheels
        // to ensure they drive forward and back correctly
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetEncoderValue (double inches) {
        // Determine new target position. Use the current position of the leftFrontMotor
        targetEncoderValue = leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
    }

    //
    //  setTargetEncoderValue MUST BE CALLED FIRST
    //  Move robot the specified number of inches at the specified speed.
    //  Positive speed moves forward; Negative speed moves backward.
    //  Encoders are not reset as the move is based on the current position.
    //  Move will stop when robot gets to desired position.
    //
    public boolean moveRobot (Direction dir, double speed)
    {
        telemetry.addData("current pos: ", rightFrontMotor.getCurrentPosition());
        telemetry.addData("target pos:  ", targetEncoderValue);
        telemetry.addData("direction:  ", dir);


        //leftFrontMotor.setTargetPosition(targetEncoderValue);
        //leftBackMotor.setTargetPosition(target);
        //rightFrontMotor.setTargetPosition(targetEncoderValue);
        //rightBackMotor.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        //leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Give power to motors
        if (dir == Direction.Forward) {
            leftFrontMotor.setPower(speed);
            leftBackMotor.setPower(speed);
            rightFrontMotor.setPower(speed);
            rightBackMotor.setPower(speed);
        }
        else if (dir == Direction.Backward) {
            leftFrontMotor.setPower(-speed);
            leftBackMotor.setPower(-speed);
            rightFrontMotor.setPower(-speed);
            rightBackMotor.setPower(-speed);
        }
        else if (dir == Direction.Right) {
            leftFrontMotor.setPower(speed);
            leftBackMotor.setPower(-speed);
            rightFrontMotor.setPower(-speed);
            rightBackMotor.setPower(speed);
        }
        else { // Direction.Left
            leftFrontMotor.setPower(-speed);
            leftBackMotor.setPower(speed);
            rightFrontMotor.setPower(-speed);
            rightBackMotor.setPower(speed);
        }


//        if (rightFrontMotor.isBusy()) {
//            return false;
//        }

        if (rightFrontMotor.getCurrentPosition() < targetEncoderValue) {
            return false;
        }

        telemetry.addData("Stopping motors", 1);

        // Motors aren't busy so we've reached our destination
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        //leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return true;
    }


    // Move robot according to desired axes motions
    //
    // Positive drive is forward (-1 to +1)
    // Positive strafe is left (-1 to +1)
    // Positive turn is counter-clockwise (-1 to +1)
    //
    public void moveRobot(double drive, double strafe, double turn) {

        // Calculate wheel powers.
        double leftFrontPower    =  drive - strafe - turn;
        double rightFrontPower   =  drive + strafe + turn;
        double leftBackPower     =  drive + strafe - turn;
        double rightBackPower    =  drive - strafe + turn;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    // Set the power of each motor to the specified value -1.0 to 1.0
    public void setMotorPower(double lf, double lb, double rf, double rb)
    {
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(lb);
        leftBackMotor.setPower(rf);
        rightBackMotor.setPower(rb);
    }

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turnRobot(double degrees){
        resetAngle();

        double error = degrees;

        while (Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
        }

        setMotorPower(0, 0, 0, 0);
    }

    public void turnRobotTo(double degrees){

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turnRobot(error);
    }

    public double getAbsoluteAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

}
