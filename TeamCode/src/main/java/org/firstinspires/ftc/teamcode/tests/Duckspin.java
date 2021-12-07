package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SPIN")
public class Duckspin extends LinearOpMode {

    public void runOpMode(){

        DcMotor spin = hardwareMap.dcMotor.get("duckSpinner");

        waitForStart();
        while (opModeIsActive()){

            spin.setDirection(DcMotorSimple.Direction.FORWARD);
            spin.setPower(-2);

        }


    }
}
