package org.firstinspires.ftc.teamcode.subsystems.Gripper;

import static org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperConstants.*;       ;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;

public class GripperSubsystem implements Subsystem {

    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    Servo servo1;
    Servo servo2;
    Servo servoClaw;
    public boolean isOpen;
    public boolean isPickup;
    public GripperSubsystem(HardwareMap map){
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
        servoClaw.setPosition(isOpen?openClaw:closeClaw);
        servo1.setPosition(isPickup?pickupAngle:0);
        servo2.setPosition(isPickup?pickupAngle:0);
        dashboard.addData("gripperOpen: ", isOpen);
    }

}
