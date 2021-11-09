package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Mechanism {
    DcMotor intakeMotor;
    @Override
    public void init(HardwareMap hardwareMap, boolean red) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.5) {
            intakeMotor.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0.5) {
            intakeMotor.setPower(-gamepad.right_trigger);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
