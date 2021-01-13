package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TestAutonomous extends LinearOpMode{
    DcMotor rampLeft;
    DcMotor rampRight;

    public void runOpMode() {

        //Setting up
        double AngleOfAttack = 15;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ramp AOA", AngleOfAttack);
        rampLeft = hardwareMap.get(DcMotor.class, "rampLeft");
        rampRight = hardwareMap.get(DcMotor.class, "rampRight");

        waitForStart();

        /*frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition();*/

        move(1);
    }

    public void move (float power) {
        rampRight.setPower(power);
        rampLeft.setPower(power);


    }
}
