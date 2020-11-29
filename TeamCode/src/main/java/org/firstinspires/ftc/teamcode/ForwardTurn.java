package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ForwardTurn", group = "Robot")
public class ForwardTurn extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Hera hera = new Hera(telemetry, hardwareMap, this);
        waitForStart();
        int counter = 1;
        while(counter < 200 && opModeIsActive()) {
            hera.forwardTurn(counter);
            counter++;
        }
    }
}
