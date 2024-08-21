package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.rustlib.commandsystem.Subsystem;
import org.rustlib.rustboard.Rustboard;

public class DroneShooter extends Subsystem {
    public final Servo angleAdjuster;
    public final Servo release;

    public DroneShooter(HardwareMap hardwareMap) {
        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjusterServo");
        release = hardwareMap.get(Servo.class, "releaseServo");
    }

    public void shootAngle() {
        angleAdjuster.setPosition(Rustboard.getDouble("shoot angle", 0.38));
    }

    public void storeAngle() {
        angleAdjuster.setPosition(Rustboard.getDouble("store angle", 0.3));
    }

    public void releaseAngle() {
        release.setPosition(Rustboard.getDouble("release angle", -0.4));
    }

    public void stopAngle() {
        release.setPosition(Rustboard.getDouble("stop angle", 0.0));
    }

}
