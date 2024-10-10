package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift;

    public void init(HardwareMap map) {
        lift = map.dcMotor.get("Lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveCC() {
        lift.setPower(-0.69);
    }

    public void moveCW() {
        lift.setPower(0.69);
    }

    public void stop() {
        lift.setPower(0.0);
    }
}
