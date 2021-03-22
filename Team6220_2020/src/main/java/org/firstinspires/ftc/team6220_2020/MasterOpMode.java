package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class MasterOpMode extends LinearOpMode
{
    //Motors
    /*DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;*/
    // Todo - move to miscallenous motors.
    public DcMotor motorLauncher;

    //Other Devices

    public void Initialize(){
        //Initialize
        //motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        //motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        //motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        //motorBackRight = hardwareMap.dcMotor.get("motorBR");
        // Todo - move to miscallenous motors.
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");


        /*motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        // Todo - move to miscallenous motors.
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

    public void driveLauncher(double power){
        motorLauncher.setPower(power);
    }

    public double getMotorSpeed(DcMotor motor, int delayInMillis) {
        double startTime = System.currentTimeMillis();
        double startPosition = motor.getCurrentPosition();
        double endTime;
        double endPosition;
        double positionChange;
        double timeChange;

        while (true) {
            if (System.currentTimeMillis() - startTime >= delayInMillis) break;
        }

        endTime = System.currentTimeMillis();
        endPosition = motor.getCurrentPosition();

        positionChange = endPosition - startPosition;
        timeChange = endTime - startTime;

        double timeChangeInMin = timeChange / 60000;
        double ticksPerMinute = positionChange/timeChangeInMin;

        if(timeChange != 0){
            return (ticksPerMinute / Constants.AM_37_TICKS_PER_ROTATION);
        } else{
            return 0;
        }

    }
}