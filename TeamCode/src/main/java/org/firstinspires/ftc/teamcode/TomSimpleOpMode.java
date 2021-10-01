package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TomSimpleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        this.waitForStart();
        if(this.opModeIsActive()) {
            while(opModeIsActive()) {
                telemetry.addData("runtime", this.getRuntime());
                telemetry.update();
            }
        }
    }
}
