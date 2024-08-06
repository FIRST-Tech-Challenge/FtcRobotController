package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gimbal {
    private AnalogInput potentiometer = null;
    private Servo gimbalPitch = null;
    private Servo gimbalYaw = null;
    private Servo gimbalPos = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public void initGimbal(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        potentiometer = hardwareMap.get(AnalogInput.class, "Analog_Port_0_CH");
        gimbalPitch = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        gimbalYaw = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        gimbalPos = hardwareMap.get(Servo.class, "Servo_Port_2_CH");
    }

    public double position() {
        return (potentiometer.getVoltage() * 81.8); // 3.3v, 270 degrees
    }

    public void telemetryGimbal() {
        telemetry.addData("Potentiometer Angle", position());
    }

    public void untuck() {
        gimbalPos.setPosition(0.71);
        gimbalPitch.setPosition(0.91);
        gimbalYaw.setPosition(0.71);
    }

    public void tuck() {
        gimbalPos.setPosition(0.9);
        gimbalPitch.setPosition(1);
        gimbalYaw.setPosition(0.9);
    }
    public void moveGimbal(){

    }

}