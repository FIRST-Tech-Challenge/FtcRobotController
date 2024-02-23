package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class bens_program extends LinearOpMode {

    private DcMotor frontLeftMotor;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                frontLeftMotor.setPower(1);
            }
        }
    }
}
