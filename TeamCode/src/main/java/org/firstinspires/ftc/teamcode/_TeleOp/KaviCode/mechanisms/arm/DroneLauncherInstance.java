package org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncherInstance {
    private HardwareMap hardwareMap;

    public Servo DroneLauncherServo;

    private double Drone_Launcher_Idle_Position = 0.5;

    private double Drone_Launcher_Launch_Position = 1;

    public void initializeDroneLauncher(HardwareMap hardwareMap) {
        DroneLauncherServo = hardwareMap.get(Servo.class, "Drone_Launcher_Servo");
        DroneLauncherServo.setDirection(Servo.Direction.FORWARD);
        DroneLauncherServo.scaleRange(0, 1);
        DroneLauncherServo.setPosition(Drone_Launcher_Idle_Position);
    }

    public void launchDrone() throws InterruptedException {

        DroneLauncherServo.setPosition(Drone_Launcher_Launch_Position);
        sleep(1500);
        DroneLauncherServo.setPosition(Drone_Launcher_Idle_Position);
    }


}
