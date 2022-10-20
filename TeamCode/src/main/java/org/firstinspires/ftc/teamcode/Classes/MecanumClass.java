package org.firstinspires.ftc.teamcode.Classes;

import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.threeten.bp.DayOfWeek;

//uttej
public class MecanumClass extends DrivetrainClass implements MotorInterface {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    // all of these methods are extended from the abstract drivetrain class, this class


    // all of these methods are extended from the abstract drivetrain class

    // need four motors(parameters) and all initialization

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

    }

    // time + speed are parameters for all the movement

    void moveForward(int time, double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveBackward(int time, double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void moveRight(int time, double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveLeft(int time, double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void turnLeft(int time, double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void turnRight(int time, double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);


    }

}