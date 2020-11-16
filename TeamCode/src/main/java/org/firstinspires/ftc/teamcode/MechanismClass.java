package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mechanism Test")

public class MechanismClass extends LinearOpMode {

    private CRServo leftConveyor, rightConveyor, intake;
    private DcMotor outtakeRight, outtakeLeft;
    private Servo flipper;

    public void runOpMode(){

        //intake and conveyor
        intake = hardwareMap.crservo.get("intake");
        leftConveyor = hardwareMap.crservo.get("leftConveyor");
        rightConveyor = hardwareMap.crservo.get("rightConveyor");

        //elevator and flipper
        //elevator = hardwareMap.crservo.get("elevator");
        flipper = hardwareMap.servo.get("flipper");

        //launcher
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");

        double intakeMod = 1.0;
        double outtakeMod = 0.0;

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }

            //Release intake

            double intakeSpeed = gamepad1.left_trigger * intakeMod;
            intake.setPower(intakeSpeed);
            rightConveyor.setPower(intakeSpeed);//turn conveyor on when the intake turns on
            leftConveyor.setPower(intakeSpeed);

            while(gamepad2.b && flipper.getPosition() <= 1){
                flipper.setPosition(flipper.getPosition() + 0.01);
            }

            while(gamepad2.a && flipper.getPosition() >= 0){
                flipper.setPosition(flipper.getPosition() - 0.01);
            }
/*
            if(gamepad2.dpad_up){
                if(outtakeMod != 1.0){
                    outtakeMod += 0.01;
                }
            }
            if(gamepad2.dpad_down){
                if(outtakeMod != 0.0){
                    outtakeMod -= 0.01;
                }
            }*/
            //Sending data on power of outtake, outtake motor RPM, and tangential velocity of outtake wheel to telemetry
            double outtakePower = (gamepad2.right_trigger * -0.5);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);

            telemetry.addData("outtake power", outtakeMod);
            telemetry.update();


        }
    }

}
