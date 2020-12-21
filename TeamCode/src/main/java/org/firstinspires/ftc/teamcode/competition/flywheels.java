package org.firstinspires.ftc.teamcode.competition;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="flywheels",group="teleop")

public class flywheels extends LinearOpMode{

    DcMotor flywheelMotorOne;
    DcMotor flywheelMotorTwo;


    @Override
    public void runOpMode() throws InterruptedException {

        //motors
        flywheelMotorOne = hardwareMap.dcMotor.get("flywheelMotorOne");
        flywheelMotorTwo = hardwareMap.dcMotor.get("flywheelMotorTwo");

        waitForStart();

        //make motors spin if button is pressed and not spin when it isn't
        while (opModeIsActive()){

            if (gamepad1.x){

                flywheelMotorOne.setPower(1);
                flywheelMotorTwo.setPower(1);

            }

            else {

                flywheelMotorOne.setPower(0);
                flywheelMotorTwo.setPower(0);

            }

        }


    }

}
