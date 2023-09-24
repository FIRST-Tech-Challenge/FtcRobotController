/*
    TODO: Add movement methods
    TODO: Add camera
    TODO: Add sensors (gyro, distance)
 */

package org.firstinspires.ftc.teamcode;

/*
    Imports
 */

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotClass {

    /*
        initializing variables
     */
    private LinearOpMode myOpMode = null;

    //defining motor variables
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public RobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(HardwareMap ahsMap) {
        //assigning motor variables to configuration name
        frontLeft = ahsMap.get(DcMotor.class, "frontLeft");
        frontRight = ahsMap.get(DcMotor.class, "frontRight");
        backLeft = ahsMap.get(DcMotor.class, "backLeft");
        backRight = ahsMap.get(DcMotor.class, "backRight");

        //setting direction of motors
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //stops all motors
    public void stopMotors() {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setting power to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveNoEncoders(double powerLeft, double powerRight, int timeInMs) throws InterruptedException {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setting power of motors
        frontLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
        //waiting while running motors
        sleep(timeInMs);
        //stops motors
        stopMotors();
    }

    //Moving using encoders
    public void move(double power, double cm) throws InterruptedException{
        //setting number of ticks per 10 cm to get number of ticks per cm
        int ticksPer10cm = 0;
        int ticksPerCm = ticksPer10cm / 10;
        int target = (int) Math.round(cm * ticksPerCm);

        //resetting
        resetEncoders();

        //setting motor mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //setting target position of motors
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        //waiting while the motors are busy
        while (frontLeft.isBusy()){
            sleep(50);
        }

        //stopping all motors
        stopMotors();
    }

    //turning with gyro code
    public void gyroTurning(double targetAngle){
    }
}
