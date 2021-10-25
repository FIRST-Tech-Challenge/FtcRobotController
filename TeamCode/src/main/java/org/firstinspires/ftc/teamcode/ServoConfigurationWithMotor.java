package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoConfigurationWithMotor extends ServoConfiguration {
    @Override
    public void runOpMode() {
        super.withMotor = true;
        super.runOpMode();
    }
}
