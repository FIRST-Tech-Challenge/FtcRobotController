package org.firstinspires.ftc.teamcode.Ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TickCounter extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");

        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rBack.setDirection(DcMotorSimple.Direction.FORWARD);

        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive()) {


            telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));
            telemetry.addLine(String.valueOf(lBack.getCurrentPosition()));
            telemetry.addLine(String.valueOf(rFront.getCurrentPosition()));
            telemetry.addLine(String.valueOf(rBack.getCurrentPosition()));

            telemetry.update();
        }
    }
}
