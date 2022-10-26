package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class wheelAttachment1 extends OpMode {
    private DcMotor WheelMotor1 = null;
    @Override
    public void init() {
        WheelMotor1 = hardwareMap.dcMotor.get("Claw1");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            WheelMotor1.setPower(1);
        }
        if (gamepad1.dpad_down){
            WheelMotor1.setPower(-1);
        }
        if (gamepad1.dpad_left){
            WheelMotor1.setPower(0);
        }
    }
}
