package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TestAutonomous extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor frontRight;

    public void runOpMode() {

        //Setting up
        double AngleOfAttack = 15;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ramp AOA", AngleOfAttack);
        frontLeft = hardwareMap.get(DcMotor.class, "rampLeft");
        frontRight = hardwareMap.get(DcMotor.class, "rampRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(360);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        frontLeft.setPower(.1);
         while (opModeIsActive() && frontLeft.isBusy()){
             telemetry.addData("hi", "hi");
             telemetry.update();
         }
         frontLeft.setPower(0);
    }
}
