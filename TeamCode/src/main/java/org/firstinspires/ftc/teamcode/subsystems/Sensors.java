package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

public class Sensors extends SubsystemBase {
    Telemetry m_telemetry;
    ColorSensor m_colorSensor;
    RevTouchSensor m_touchSensor;



    public Sensors(ColorSensor colorSensor, RevTouchSensor touchSensor, Telemetry telemetry) {
        m_touchSensor = touchSensor;
        m_colorSensor = colorSensor;
        m_telemetry = telemetry;


        m_telemetry.addLine("Sensors Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.addData("Current Color", getColorAsString());
        m_telemetry.addData("Touch Sensor Pressed", isPressed());
        m_telemetry.update();
    }

    public String getColorAsString() {
        return String.format("Red: %d, Blue: %d, Green: %d",
                m_colorSensor.red(), m_colorSensor.blue(), m_colorSensor.red());
    }

    public boolean isPressed() {
        return m_touchSensor.isPressed();
    }
}
