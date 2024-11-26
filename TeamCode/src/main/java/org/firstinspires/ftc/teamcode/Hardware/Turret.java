package org.firstinspires.ftc.teamcode.Hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    private final Servo turret;
    public double lastPosition = 0;


    public Turret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(Servo.class, "turret");
    }

    public void setPosition(double position) {
        if (position != lastPosition) {
            turret.setPosition(position);
        }
        lastPosition = position;
    }
}
