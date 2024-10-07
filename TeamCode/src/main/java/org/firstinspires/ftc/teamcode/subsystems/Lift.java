package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift extends LinearOpMode{
    DcMotor lyft;
    @Override
    public void runOpMode() {
        lyft = hardwareMap.get(DcMotor.class, "lyft");

        waitForStart();
//        while(opModeIsActive()){}
        while(!isStopRequested()){
            lyft.setPower(gamepad1.right_trigger);
            telemetry.addData("Lift Position", lyft.getCurrentPosition());
            telemetry.update();
        }
    }
}
