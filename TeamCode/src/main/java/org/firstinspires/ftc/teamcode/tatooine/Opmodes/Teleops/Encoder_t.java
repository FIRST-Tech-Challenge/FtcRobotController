package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Encoder_t extends LinearOpMode {

    DcMotor encoder1;

    @Override public void runOpMode(){

        encoder1 = hardwareMap.get(DcMotor.class, "encoder1");

        waitForStart();

        while(opModeIsActive()){

            encoder1.setPower(1);

            telemetry.addData("encoder1", encoder1.getCurrentPosition());
            telemetry.update();
        }
    }


}
