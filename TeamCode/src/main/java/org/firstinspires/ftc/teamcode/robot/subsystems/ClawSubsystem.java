package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSubsystem extends SubsystemBase
{
    private static final double CLAW_OPEN = 0.7;
    private static final double CLAW_CLOSED = 0;

    public static final double DETECTION_DISTANCE = 40;

    private static Servo clawLeft;
    private static Servo clawRight;
    private static DistanceSensor leftDist;
    private static DistanceSensor rightDist;

    public ClawSubsystem(final HardwareMap hMap) {
        clawLeft = hMap.get(Servo.class, "claw_left");
        clawRight = hMap.get(Servo.class, "claw_right");
        leftDist = hMap.get(DistanceSensor.class, "leftDist");
        rightDist = hMap.get(DistanceSensor.class, "rightDist");
    }

    public void open() {
        clawLeft.setPosition(CLAW_OPEN);
        clawRight.setPosition(CLAW_OPEN);
    }

    public void close() {
        clawLeft.setPosition(CLAW_CLOSED);
        clawRight.setPosition(CLAW_CLOSED);
    }

    public boolean pixelDetected()
    {
        return (leftDist.getDistance(MM)<DETECTION_DISTANCE);
    }
}