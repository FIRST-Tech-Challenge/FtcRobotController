package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class MasterOpMode extends LinearOpMode
{
    //Motors
    /*public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;*/
    // Todo - move to miscellaneous motors.
    public static DcMotor motorLauncher;

    //Other Devices

    public void Initialize(){
        //Initialize
        //motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        //motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        //motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        //motorBackRight = hardwareMap.dcMotor.get("motorBR");
        // Todo - move to miscellaneous motors.
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");


        /*motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        // Todo - move to miscellaneous motors.
        motorLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*public void driveMecanum(double driveAngle, double drivePower, double w)
    {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        double motorFLPower = x + y + -w;
        double motorFRPower = x + -y + -w;
        double motorBLPower = -x + y + -w;
        double motorBRPower = -x + -y + -w;

        double scaleFactor = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));

        if(scaleFactor > 1){
            motorFrontLeft.setPower(motorFLPower / scaleFactor);
            motorFrontRight.setPower(motorFRPower / scaleFactor);
            motorBackLeft.setPower(motorBLPower / scaleFactor);
            motorBackRight.setPower(motorBRPower / scaleFactor);
        } else {
            motorFrontLeft.setPower(motorFLPower);
            motorFrontRight.setPower(motorFRPower);
            motorBackLeft.setPower(motorBLPower);
            motorBackRight.setPower(motorBRPower);
        }
    }*/

    //Sets the launch motor to a given power
    public void driveLauncher(double power){
        motorLauncher.setPower(power);
    }

    //This method returns the speed of a given motor after a delay od delayInMillis
    //@param motor Input the motor you want to know the RPM of
    //@param delayInMillis Input the delay you want to measure the change in encoder ticks in milliseconds.
    public double getMotorTicksPerMinute(DcMotor motor, int delayInMillis) {

        //Variables used only in this method
        double startTime = System.currentTimeMillis();
        double startPosition = motor.getCurrentPosition();
        double endTime;
        double endPosition;
        double positionChange;
        double timeChange;

        //Waits delayInMillis milliseconds before recording endTime and endPosition
        while (true) {
            if (System.currentTimeMillis() - startTime >= delayInMillis) break;
        }

        endTime = System.currentTimeMillis();
        endPosition = motor.getCurrentPosition();

        //Calculates the ▲Position and the ▲Time
        positionChange = endPosition - startPosition;
        timeChange = endTime - startTime;

        //Converts the ▲Time from milliseconds to minutes then finds encoder ticks per minute
        double timeChangeInMin = timeChange / 60000;

        //To avoid divide by zero we need to be sure timeChangeInMin does not equal zero.
        double ticksPerMinute = 0;
        if(timeChangeInMin != 0) {
            ticksPerMinute = positionChange / timeChangeInMin;
        }

        //If timeChange is not zero return the motor RPM otherwise return zero.
        return (ticksPerMinute);

    }
}