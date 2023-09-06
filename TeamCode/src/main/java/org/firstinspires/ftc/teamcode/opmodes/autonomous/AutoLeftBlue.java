package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.hardware.LightBlue;
import org.firstinspires.ftc.teamcode.hardware.Lights;

@Autonomous(name = "Auto-Left-BLUE")
public class AutoLeftBlue extends AutoLinearBase {
    @Override
    public int side() {
        return LEFT_SIDE;
}

    @Override
    public double turnDirection() {
        return this.LEFT;
    }

    @Override
    public ColorSensorDevice getColorSensorDevice() {
        return colorSensorDeviceRight;
    }

    @Override
    public void strafeDirection(double distance) {
        driveRight(distance);
    }

    @Override
    public void strafeAntiDirection(double distance) {
        driveLeft(distance);
    }

    @Override
    public void lightOn() {
        light.blueOn();
    }
    @Override
    public Lights getLight() {
        return new LightBlue(hardwareMap.dcMotor.get("LIGHTS"));
    }
}