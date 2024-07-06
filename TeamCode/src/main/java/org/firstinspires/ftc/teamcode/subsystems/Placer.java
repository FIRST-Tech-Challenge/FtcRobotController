package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.rustlib.commandsystem.Subsystem;
import org.rustlib.rustboard.RustboardServer;

public class Placer extends Subsystem {
    public final TouchSensor touchSensor;
    private final Servo lifter0;
    private final Servo lifter1;
    private final Servo placer;
    public DistanceSensor distanceSensor;

    public Placer(HardwareMap hardwareMap) {
        lifter0 = hardwareMap.get(Servo.class, "lifter0");
        lifter0.setDirection(Servo.Direction.FORWARD);
        lifter0.close();
        lifter1 = hardwareMap.get(Servo.class, "lifter1");
        lifter1.setDirection(Servo.Direction.FORWARD);
        lifter1.close();
        placer = hardwareMap.get(Servo.class, "placer");
        placer.setDirection(Servo.Direction.FORWARD);
        placer.close();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "placerTouch");
    }

    public void placePosition() {
        lifter0.setPosition(SubsystemConstants.Placer.lifter0PlacePosition);
        lifter1.setPosition(SubsystemConstants.Placer.lifter1PlacePosition);
    }

    public void storagePosition() {
        lifter0.setPosition(SubsystemConstants.Placer.lifter0StoragePosition);
        lifter1.setPosition(SubsystemConstants.Placer.lifter1StoragePosition);
        placer.setPosition(SubsystemConstants.Placer.closePosition);
    }

    public void open() {
        placer.setPosition(SubsystemConstants.Placer.openPosition);
    }

    public void close() {
        placer.setPosition(SubsystemConstants.Placer.closePosition);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void periodic() {
        RustboardServer.setNodeValue("distance", getDistance());
    }
}
