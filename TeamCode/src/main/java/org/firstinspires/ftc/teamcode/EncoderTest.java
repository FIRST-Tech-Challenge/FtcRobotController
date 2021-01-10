package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Encoder Test")
public class EncoderTest extends LinearOpMode {

    private DcMotor encoder;

    @Override
    public void runOpMode(){
        encoder = hardwareMap.dcMotor.get("encoder");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Encoder position", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
