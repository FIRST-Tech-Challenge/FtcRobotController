package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class BasicChassis {

    public enum ChassisType {
        ENCODER,IMU,ODOMETRY,VSLAM
    }
    //initialize motor
    protected final DcMotorEx motorLeftFront;
    protected final DcMotorEx motorRightFront;
    protected final DcMotorEx motorLeftBack;
    protected final DcMotorEx motorRightBack;

    protected final double robot_diameter = Math.sqrt(619.84);
    protected final double wheel_diameter = 3.93701;
    protected final double[] encoder = new double[4];
    protected double xpos = 0;
    protected double ypos = 0;

    protected final double counts_per_motor_goBilda = 383.6;
    protected final double counts_per_inch = 2 * (counts_per_motor_goBilda / (wheel_diameter * Math.PI));

    public BasicChassis() {
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
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    abstract public void setRightMotorPowers(double power);
    abstract public void setLeftMotorPowers(double power);

    abstract public void setMotorPowers(double power);
    abstract public void stopAllMotors();
    abstract public void turnInPlace(double target, double power);
    abstract public void moveForward(double distance, double power) ;
    abstract public void moveBackward(double distance, double power);
    abstract public void moveRight(double distance, double power);
    abstract public void moveLeft(double distance, double power);
    abstract public void moveAngle(double x, double y, double power);
    abstract public void setPosition(double xPosition, double yPosition, double newangle);
    abstract public void goToPosition(int direction, double xPosition, double yPosition, double newangle, double power);
    abstract public boolean goToPositionTeleop(int direction, double xPosition, double yPosition, double power);
    abstract public void goToPositionWithoutStop(int direction, double xPosition, double yPosition, double power);
    abstract public void navigate();
    abstract public void navigateTeleOp();
    abstract public double[] track();

    abstract public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power, double targetAnglu);
    abstract public void tripleSplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power);
    abstract public void partOfPolySplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double targetAnglu, double power);
    abstract public void partOfPolySplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power);
    //replace with move forward or backward at different speeds
    public abstract void setRightMotorVelocities(double velocity);
    public abstract void setLeftMotorVelocities(double velocity);

    public void moveMultidirectional(double power, double angle, float rightStick, boolean isSlow) {
        double angleInRadian;
        angleInRadian = Math.toRadians(angle);
        rightStick*=-1;
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightStick*=0.65
        ;
        if (isSlow) {
            if(abs(rightStick)<0.999&abs(rightStick)>0.2){
                rightStick=0.999f*abs(rightStick)/rightStick;
            }
            motorLeftBack.setPower((power + rightStick) * 0.3);
            motorRightFront.setPower((power - rightStick) * 0.3);
            motorRightBack.setPower((power - rightStick) * 0.3);
            motorLeftFront.setPower((power + rightStick) * 0.3);
        } else {
            motorLeftBack.setPower(power + rightStick);
            motorRightFront.setPower(power - rightStick);
            motorRightBack.setPower(power - rightStick);
            motorLeftFront.setPower(power + rightStick);
        }
        track();
    }

    public void moveMultidirectionalMecanum(double power, float leftStickx, float leftSticky, float rightStick) {
//        rightStick*=-1;
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftStickx*=0.65;
//        leftSticky*=0.65;

        double powera = 0;
        double powerb = 0;

        if ((leftStickx + leftSticky)/(leftSticky - leftStickx) >= 1) {
            powera = 1;
            powerb = (leftSticky - leftStickx)/(leftStickx + leftSticky);
        }
        else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) <= -1) {
            powera = -1;
            powerb = -(leftSticky - leftStickx)/(leftStickx + leftSticky);
        }
        else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) > -1 && (leftStickx + leftSticky)/(leftSticky - leftStickx) < 0 || leftSticky-leftStickx < 0 && (leftStickx + leftSticky) == 0) {
            powerb = -1;
            powera = -(leftStickx + leftSticky)/(leftSticky - leftStickx);
        }
        else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) < 1 && (leftStickx + leftSticky)/(leftSticky - leftStickx) > 0 || leftSticky-leftStickx > 0 && (leftStickx + leftSticky) == 0) {
            powerb = 1;
            powera = (leftStickx + leftSticky)/(leftSticky - leftStickx);
        }


//        if (isSlow) {
//            if(abs(rightStick)<0.999&abs(rightStick)>0.2){
//                rightStick=0.999f*abs(rightStick)/rightStick;
//            }
//            motorLeftBack.setPower(rightSticky * 1.25/Math.sqrt(2));
//            motorRightFront.setPower((power - rightStick) * 0.3);
//            motorRightBack.setPower((power - rightStick) * 0.3);
//            motorLeftFront.setPower((power + rightStick) * 0.3);
//        }
//        else {
            motorLeftFront.setPower(powera-rightStick);
            motorRightBack.setPower(powera+rightStick);

            motorRightFront.setPower(powerb+rightStick);
            motorLeftBack.setPower(powerb-rightStick);
//        }
        track();
    }
}