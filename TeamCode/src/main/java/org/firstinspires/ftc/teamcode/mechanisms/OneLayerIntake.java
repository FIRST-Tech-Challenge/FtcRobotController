package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OneLayerIntake {
    private DcMotor outer;


    public void init(HardwareMap hardwareMap) {
        outer = hardwareMap.get(DcMotor.class, "outer_intake_motor");
        outer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outer.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intake() {
        // these values can be changed
        outer.setPower(0.4);
    }

    public void stop() {
        outer.setPower(0.0);
    }

    public void garden() {
        // these values can be changed
        outer.setPower(-0.4);
    }
}
