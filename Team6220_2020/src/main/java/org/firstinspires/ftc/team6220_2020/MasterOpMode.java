package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.DriverInput;

public abstract class MasterOpMode extends LinearOpMode
{
    //Motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    //Other Devices

    // Create drivers
    public DriverInput driver1;
    public DriverInput driver2;

    //This method initializes the motors.
    public void Initialize(){
        //Initialize
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //This method drives mecanum when given an angle drive power and turning power
    public void driveMecanum(double driveAngle, double drivePower, double turningPower)
    {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        double motorFLPower = x + y + -turningPower;
        double motorFRPower = x + -y + -turningPower;
        double motorBLPower = -x + y + -turningPower;
        double motorBRPower = -x + -y + -turningPower;

        double scaleFactor = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));

        if(scaleFactor > 1){
            motorFL.setPower(motorFLPower / scaleFactor);
            motorFR.setPower(motorFRPower / scaleFactor);
            motorBL.setPower(motorBLPower / scaleFactor);
            motorBR.setPower(motorBRPower / scaleFactor);
        } else {
            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);
        }

    }


}

