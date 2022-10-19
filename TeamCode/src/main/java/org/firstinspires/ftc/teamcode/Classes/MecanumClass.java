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
    void moveForward(time,speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setpower(speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void moveBackward(time,speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        frontRight.setpower(-speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);
    }

    void moveRight(time,speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setpower(speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void moveLeft(time,speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setpower(-speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);
    }

    void turnLeft(time,speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setpower(speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);

    }

    void turnRight(time,speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setpower(-speed);

        sleep(time*1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setpower(0);


}
