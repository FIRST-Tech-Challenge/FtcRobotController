package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intake;
    private DcMotor intake1;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
    }

    public void run(double pow){
        intake.setPower(pow);
        intake1.setPower(-pow);
    }
}
