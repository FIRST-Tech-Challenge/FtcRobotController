package org.firstinspires.ftc.teamcode.subsystems.Gripper;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

public class gripperSubsystem implements Subsystem {

    private HardwareMap map;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    Servo servo1;
    Servo servo2;
    Servo servoClaw;
    private boolean isOpen;

    public gripperSubsystem(HardwareMap map){
        servo1 = map.servo.get("servo1");
        servo2 = map.servo.get("servo2");
        servoClaw = map.servo.get("servoClaw");
        register();
        servo1.getController().pwmEnable();
        servo2.getController().pwmEnable();
        servoClaw.getController().pwmEnable();
        isOpen = false;

    }
    public void periodic() {

    }



}
