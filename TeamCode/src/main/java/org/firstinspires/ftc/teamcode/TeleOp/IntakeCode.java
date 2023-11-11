package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeCode extends LinearOpMode{

    private DcMotor Bleft;


    @Override
    public void runOpMode() {
        Bleft = hardwareMap.dcMotor.get("Bleft");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0.0) {

                //Goes forward
                Bleft.setPower(0.5*gamepad1.left_stick_y);

            }



            else{
                Bleft.setPower(0);

            }

        }







    }
}

