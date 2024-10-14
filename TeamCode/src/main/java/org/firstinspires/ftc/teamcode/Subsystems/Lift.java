package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift1;
    public DcMotor lift2;

    public void init(HardwareMap map) {
        lift1 = map.dcMotor.get("Lift1");
        lift2 = map.dcMotor.get("Lift2");
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveCC() {
        lift1.setPower(-1);
        lift2.setPower(-1);
    }

    public void moveCW() {
        lift2.setPower(1);
        lift1.setPower(1);
    }

    public void stop() {
        lift1.setPower(0);
        lift2.setPower(0);

    }
}
