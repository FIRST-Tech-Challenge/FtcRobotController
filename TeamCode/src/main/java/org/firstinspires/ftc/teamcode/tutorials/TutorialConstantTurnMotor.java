// Date: 7-16-24

package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tutorial: Constant Turns Motor")
public class TutorialConstantTurnMotor extends OpMode {
    DcMotor motor;
    double power;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motory");
        power = 0.3141592653;
    }

    @Override
    public void loop() {
        motor.setPower(power);
    }
}