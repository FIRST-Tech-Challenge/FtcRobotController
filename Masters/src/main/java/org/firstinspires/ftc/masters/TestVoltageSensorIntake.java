package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.masters.util.LynxModuleUtil;

public class TestVoltageSensorIntake extends LinearOpMode {

    DcMotor intake;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        while (opModeIsActive()) {

            telemetry.addData("voltage", batteryVoltageSensor.getVoltage());
            intake.setPower(1);
            telemetry.update();


        }

    }
}
