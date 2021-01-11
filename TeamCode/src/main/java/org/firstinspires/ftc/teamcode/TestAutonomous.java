package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TestAutonomous extends LinearOpMode{
    DcMotor ramp;

    public void runOpMode() {

        //Setting up
        double AOA = 15;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ramp AOA", AOA);
        ramp = hardwareMap.get(DcMotor.class, "frontLeft");

        waitForStart();

        /*frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition();*/

        move(1);
    }

    public void move (double power) {
        ramp.setPower(power);

    }
}
