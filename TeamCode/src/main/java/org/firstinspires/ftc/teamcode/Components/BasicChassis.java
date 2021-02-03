package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BasicChassis {
    public enum ChassisType {
        ENCODER,IMU,ODOMETRY
    }
    //initialize motor
    protected DcMotorEx motorLeftFront;
    protected DcMotorEx motorRightFront;
    protected DcMotorEx motorLeftBack;
    protected DcMotorEx motorRightBack;

    protected final double robot_diameter = Math.sqrt(619.84);
    protected final double wheel_diameter = 3.93701;
    protected double[] encoder = new double[4];
    protected double xpos = 0;
    protected double ypos = 0;

    protected final double counts_per_motor_goBilda = 383.6;
    protected final double counts_per_inch = 2 * (counts_per_motor_goBilda / (wheel_diameter * Math.PI));
    protected LinearOpMode op = null;

    public BasicChassis(LinearOpMode opMode) {
        op = opMode;
        //lastAngles  = new Orientation();
        // Chassis motors
        motorLeftFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");

        // Chassis Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    abstract public void stopAllMotors();
    abstract public void turnInPlace(double target, double power);
    abstract public void moveForward(double distance, double power) ;
    abstract public void moveBackward(double distance, double power);
    abstract public void moveRight(double distance, double power);
    abstract public void moveLeft(double distance, double power);
    abstract public void moveAngle(double x, double y, double power);
    abstract public void setPosition(double xPosition, double yPosition, double newangle);
    abstract public void goToPosition(double xPosition, double yPosition, double newangle, double power);
    abstract public void navigate();
    public void moveMultidirectional(double power, double angle, float rightStick, boolean isSlow) {
        double angleInRadian;
        angleInRadian = Math.toRadians(angle);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (isSlow) {
            motorLeftBack.setPower((Math.sin(angleInRadian - Math.PI / 4) * power - rightStick) * 0.3);
            motorRightBack.setPower((Math.sin(angleInRadian + Math.PI / 4) * power + rightStick) * 0.3);
            motorLeftFront.setPower((Math.sin(angleInRadian + Math.PI / 4) * power - rightStick) * 0.3);
            motorRightFront.setPower((Math.sin(angleInRadian - Math.PI / 4) * power + rightStick) * 0.3);
        } else {
            motorLeftBack.setPower(Math.sin(angleInRadian - Math.PI / 4) * power - rightStick);
            motorRightBack.setPower(Math.sin(angleInRadian + Math.PI / 4) * power + rightStick);
            motorLeftFront.setPower(Math.sin(angleInRadian + Math.PI / 4) * power - rightStick);
            motorRightFront.setPower(Math.sin(angleInRadian - Math.PI / 4) * power + rightStick);
        }
    }

}