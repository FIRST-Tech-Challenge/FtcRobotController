package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class PlaneLauncherSubsystem extends SubsystemBase
{
    public static final double PLANE_OPEN = 0.7;
    public static final double PLANE_CLOSE = 0;

    private static Servo planeServo;

    public PlaneLauncherSubsystem(final HardwareMap hMap) {
        planeServo = hMap.get(Servo.class, "plane");
        planeServo.setPosition(PLANE_CLOSE);
    }

    public void shoot() {
        planeServo.setPosition(PLANE_OPEN);
    }
}