package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSubsystem extends SubsystemBase
{
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW1_OPEN = 0.6;
    private static final double CLAW_CLOSED = 0.5;
    private static final double CLAW1_CLOSED = 0.32;

    public static final double DETECTION_DISTANCE = 27;

    private static Servo clawLeft;
    private static Servo clawRight;
    private static DistanceSensor leftDist;
    private static DistanceSensor rightDist;

    public ClawSubsystem(final HardwareMap hMap) {
        clawLeft = hMap.get(Servo.class, "claw1");
        clawRight = hMap.get(Servo.class, "claw");
        leftDist = hMap.get(DistanceSensor.class, "leftDist");
        rightDist = hMap.get(DistanceSensor.class, "rightDist");
    }

    public void open() {
        clawLeft.setPosition(CLAW1_OPEN);
        clawRight.setPosition(CLAW_OPEN);
    }

    public void close() {
        clawLeft.setPosition(CLAW1_CLOSED);
        clawRight.setPosition(CLAW_CLOSED);
    }

    public boolean pixelDetected()
    {
        return (leftDist.getDistance(MM)<DETECTION_DISTANCE);
    }
}