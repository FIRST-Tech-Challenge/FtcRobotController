package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="Dead Wheel Test")

public class DeadWheelTest extends LinearOpMode {

    DcMotor encoder1 = null;
    DcMotor encoder2 = null;
    DcMotor encoder3 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        encoder1 = hardwareMap.dcMotor.get("encoder1");
        encoder2 = hardwareMap.dcMotor.get("encoder2");
        encoder3 = hardwareMap.dcMotor.get("encoder3");

        encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        while (opModeIsActive()) {

            int position1 = encoder1.getCurrentPosition();
            int position2 = encoder2.getCurrentPosition();
            int position3 = encoder3.getCurrentPosition();
            position2*=-1;

            telemetry.addData("Position 1", position1);
            telemetry.addData("Position 2", position2);
            telemetry.addData("Position 3", position3);
            telemetry.update();

        }
    }
}
