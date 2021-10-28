package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestOuttake extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        boolean FlapOpenOrClosed = true;
        Outtake outtake = new Outtake (hardwareMap);


        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (FlapOpenOrClosed) {
                outtake.FlapOpen();
                }

                else {
                outtake.FlapClosed();

                }

            }
        }
    }
}
