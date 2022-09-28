package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Localizer;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOp extends OpMode

{
    private Localizer localizer = null;
    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new Localizer(hardwareMap);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        telemetry.update();
    }
}
