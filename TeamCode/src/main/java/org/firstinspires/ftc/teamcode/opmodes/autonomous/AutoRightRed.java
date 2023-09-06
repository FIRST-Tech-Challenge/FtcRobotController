package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LightRed;
import org.firstinspires.ftc.teamcode.hardware.Lights;

@Autonomous(name = "Auto-Right-RED")
public class AutoRightRed extends AutoLinearBase {
    @Override
    public int side() {
        return RIGHT_SIDE;
    }

    @Override
    public double turnDirection() {
        return this.RIGHT;
    }

    @Override
    public ColorSensorDevice getColorSensorDevice() {
        return colorSensorDeviceLeft;
    }

    @Override
    public void strafeDirection(double distance) {
        driveLeft(distance);
    }

    @Override
    public void strafeAntiDirection(double distance) {
        driveRight(distance);
    }

    @Override
    public void lightOn() {
        light.redOn();
    }
    @Override
    public Lights getLight() {
        return new LightRed(hardwareMap.dcMotor.get("LIGHTS"));
    }
}