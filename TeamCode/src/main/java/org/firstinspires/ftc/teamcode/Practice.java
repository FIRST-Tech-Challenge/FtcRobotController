package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class Practice extends OpMode {
    @Override
    public void init(){

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;

        boolean touchSense_1 = false;
        String ID = "2920jaj";

        telemetry.addData("Power level", leftStickX);

    }

    @Override
    public void loop(){

    }
}
