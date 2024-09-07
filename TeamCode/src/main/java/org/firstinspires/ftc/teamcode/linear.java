package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linear extends OpMode {

    public DcMotorEx linear_motion;

    @Override
    public void init() {
        linear_motion = hardwareMap.get(DcMotorEx.class, "linear_motion");

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            linear_motion.setPower(.3);
        }
        else if (gamepad1.dpad_down){
            linear_motion.setPower(-.3);
        }

    }

    public void init(HardwareMap hardwareMap) {
    }
}
