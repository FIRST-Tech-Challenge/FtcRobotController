package org.firstinspires.ftc.teamcode.epicDemo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class asdjfklsadjfk extends LinearOpMode {
    public ExternalTelemetryClass externalTelemetryClass;
    public DcMotor dcMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        externalTelemetryClass = new ExternalTelemetryClass(telemetry, hardwareMap);
        waitForStart();
        telemetry.addLine("asdfasdfasdfasdf is being run");
        externalTelemetryClass.kennansEpicFunction();
        telemetry.update();
    }
}
