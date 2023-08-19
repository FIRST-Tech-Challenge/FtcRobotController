package org.firstinspires.ftc.teamcode.Ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TickCounter extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        while (opModeIsActive()) {
            telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));
            telemetry.update();
        }
    }
}
