package org.firstinspires.ftc.teamcode.Classes;

import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//uttej
public class MecanumClass extends DrivetrainClass implements MotorInterface {

    // all of these methods are extended from the abstract drivetrain class, this class


    // need four motors(parameters) and all initialization

    void initialize(String frontLeft, String frontRight, String backLeft, String backRight) {
        frontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"BackRight");

        }

    }

    // time + speed are parameters for all the movement
    void moveForward(int time,double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setpower(speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void moveBackward(int time,double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        frontRight.setpower(-speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);
    }

    void moveRight(int time,double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setpower(speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void moveLeft(int time,double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setpower(-speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);
    }

    void turnLeft(int time,double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setpower(speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void turnRight(int time,double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setpower(-speed);

        thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);


}
