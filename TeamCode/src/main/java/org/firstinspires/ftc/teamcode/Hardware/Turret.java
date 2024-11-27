package org.firstinspires.ftc.teamcode.Hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//perpendicular counterclockwise: .17, perpendicular clockwise: .84. middle: .5
public class Turret {
    private final Servo turret;
    public double lastPosition = 0;
    public static final double MIN = .17;
    public static final double MAX = .84;

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
