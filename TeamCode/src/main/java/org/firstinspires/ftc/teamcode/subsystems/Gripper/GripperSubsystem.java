package org.firstinspires.ftc.teamcode.subsystems.Gripper;

import static org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperConstants.*;       ;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;

public class GripperSubsystem extends SubsystemBase {

    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    Servo servoClaw;
    Servo servo2;
    Servo servo1;
    public boolean isOpen;
    public boolean isPickup;
    public GripperSubsystem(HardwareMap map){
        servo1 = map.servo.get("servoClaw");
        servo2 = map.servo.get("servo2");
        servoClaw = map.servo.get("servo1");
        register();
        servoClaw.getController().pwmEnable();
        servo2.getController().pwmEnable();
        servoClaw.getController().pwmEnable();
        isOpen = false;

    }
    public void periodic() {
        servo1.setPosition(isOpen?openClaw:closeClaw);
        servoClaw.setPosition(isPickup?pickupAngle:0);
        servo2.setPosition(isPickup?pickupAngle:0);
        dashboard.addData("gripperOpen: ", isOpen);
    }
    public Command CloseGripper(){
        return new InstantCommand(()->isOpen = false);
    }
    public Command OpenGripper(){
        return new InstantCommand(()->isOpen = true);
    }
    public Command setPickup(){
        return new InstantCommand(()->isPickup = true);
    }
}
