package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "armEncoderTest")
public class armEncoderTest extends OpMode{

    Telemetry mTelmeurytrue = FtcDashboard.getInstance().getTelemetry();

    AnalogInput sensor;
    @Override
    public void init() {
          sensor = hardwareMap.analogInput.get("ARM_ENC");
    }

    @Override
    public void loop(){
         telemetry.addData("Voltage: ", sensor.getVoltage());
         telemetry.addData("Max voltage: ", sensor.getMaxVoltage());
         telemetry.update();
    }
}
