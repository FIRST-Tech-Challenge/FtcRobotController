package org.firstinspires.ftc.teamcode.others;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimpleJoystickMotorSpinOpmode extends OpMode {
    DcMotor motor1;
    double power = 0.5;
    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        motor1.setPower(power);

    }
}
