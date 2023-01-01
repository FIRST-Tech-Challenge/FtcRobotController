package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class CompHardware {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor rightSlides = null;
    public DcMotor leftSlides = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;

    public BNO055IMU imu = null;
    Orientation lastAngles = null;
    private double globalAngle = 0.0;
    public boolean encoder;


    // Other variable names
    HardwareMap hwMap;

    public CompHardware(boolean encoder) {
        hwMap = null;
        this.encoder = encoder; //setting if you want to run with encoders
    }

    //called in initializeRobot method in AutonomousMethods
    public void initializeHardware(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define Motors
        frontLeftMotor = hwMap.dcMotor.get("front_left");

        frontRightMotor = hwMap.dcMotor.get("front_right");

        backLeftMotor = hwMap.dcMotor.get("back_left");

        backRightMotor = hwMap.dcMotor.get("back_right");

        rightSlides = hwMap.dcMotor.get("rightSlides");

        leftSlides = hwMap.dcMotor.get("leftSlides");


        // Define Servos
        leftIntake = hwMap.crservo.get("leftIntake");

        rightIntake = hwMap.crservo.get("rightIntake");

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        //Drivetrain Motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Originally Reverse
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);//Originally Forward
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); //Originally Reverse
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);//Originally Forward

        //Subsystem Motors & Servos
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lastAngles = new Orientation();

        if(encoder) {
            // May use RUN_USING_ENCODERS if encoders are installed
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Setting Motor/Servo Powers
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }

    public void encoderDrive(String direction, double centimeters, double power) {
        //ticks is equal to (distance / circumference) * (encoder counts per wheel revolution)
        if (encoder = false) {
            throw new IllegalArgumentException("Encoder not set to true");
        }
        int ticks = (int) ((centimeters / (Math.PI * 9.6)) * 537.7);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Switched from original Hardware class
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Originally Reverse
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);//Originally Forward
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD); //Originally Reverse
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);//Originally Forward

        if (direction.equals("Front")) {
            int rightFrontTargetPos = frontRightMotor.getCurrentPosition() + ticks;
            int leftFrontTargetPos = frontLeftMotor.getCurrentPosition() + ticks;
            int rightBackTargetPos = backRightMotor.getCurrentPosition() + ticks;
            int leftBackTargetPos = backLeftMotor.getCurrentPosition() + ticks;

            frontRightMotor.setTargetPosition(rightFrontTargetPos);
            frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            backRightMotor.setTargetPosition(rightBackTargetPos);
            backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Back")) {
            int rightFrontTargetPos = frontRightMotor.getCurrentPosition() - ticks;
            int leftFrontTargetPos = frontLeftMotor.getCurrentPosition() - ticks;
            int rightBackTargetPos = backRightMotor.getCurrentPosition() - ticks;
            int leftBackTargetPos = backLeftMotor.getCurrentPosition() - ticks;

            frontRightMotor.setTargetPosition(rightFrontTargetPos);
            frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            backRightMotor.setTargetPosition(rightBackTargetPos);
            backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Right")) {
            int rightFrontTargetPos = frontRightMotor.getCurrentPosition() - 2 * ticks;
            int leftFrontTargetPos = frontLeftMotor.getCurrentPosition() + 2 * ticks;
            int rightBackTargetPos = backRightMotor.getCurrentPosition() + 2 * ticks;
            int leftBackTargetPos = backLeftMotor.getCurrentPosition() - 2 * ticks;

            frontRightMotor.setTargetPosition(rightFrontTargetPos);
            frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            backRightMotor.setTargetPosition(rightBackTargetPos);
            backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Left")){
            int rightFrontTargetPos = frontRightMotor.getCurrentPosition() + 2 * ticks;
            int leftFrontTargetPos = frontLeftMotor.getCurrentPosition() - 2 * ticks;
            int rightBackTargetPos = backRightMotor.getCurrentPosition() - 2 * ticks;
            int leftBackTargetPos = backLeftMotor.getCurrentPosition() + 2 * ticks;

            frontRightMotor.setTargetPosition(rightFrontTargetPos);
            frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            backRightMotor.setTargetPosition(rightBackTargetPos);
            backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setPowerOfAllMotorsTo(0);
        }
    }

    public void setPowerOfAllMotorsTo(double power) {
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);
    }

    public void setPowerOfIndividualMotorsTo(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setPower(frontRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setPower(rightPower);
        frontLeftMotor.setPower(leftPower);
        backRightMotor.setPower(rightPower);
        backLeftMotor.setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
//                /*telemetry.addData("Direction", checkDirection());
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

            while (getAngle() > degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }
        }
        else    // left turn.
            while (getAngle() < degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

        // turn the motors off.
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void sleep(double time){
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds()<(time/1000)){

        }

    }
    public void moveSlide (int height, double speed) {
        rightSlides.setTargetPosition(height);
        leftSlides.setTargetPosition(height);
        rightSlides.setPower(speed);
        leftSlides.setPower(speed);
        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void intake(){
        rightIntake.setPower(-1);
        leftIntake.setPower(1);

    }
    public void outake(){
        rightIntake.setPower(1);
        leftIntake.setPower(-1);
    }
    public void stopIntake(){
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    }

    /* public void stopOuttake(){
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    } */
}