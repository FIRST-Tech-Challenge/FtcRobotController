package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.rustlib.commandsystem.Subsystem;
import org.rustlib.rustboard.RustboardServer;

public class DroneShooter extends Subsystem {
    public final Servo angleAdjuster;
    public final Servo release;

    public DroneShooter(HardwareMap hardwareMap) {
        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjusterServo");
        release = hardwareMap.get(Servo.class, "releaseServo");
    }

    public void shootAngle() {
        angleAdjuster.setPosition(RustboardServer.getDoubleValue("shoot angle", 0.38));
    }

    public void storeAngle() {
        angleAdjuster.setPosition(RustboardServer.getDoubleValue("store angle", 0.3));
    }

    public void releaseAngle() {
        release.setPosition(RustboardServer.getRustboardLayout("dashboard_0").getDoubleValue("release angle", -0.4));
    }

    public void stopAngle() {
        release.setPosition(RustboardServer.getDoubleValue("stop angle", 0.0));
    }

}
