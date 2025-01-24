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
    Servo rotServo;
    public boolean isOpen;
    public boolean isPickup;
    public GripperSubsystem(HardwareMap map){
        rotServo = map.servo.get("rotServo");
        servoClaw = map.servo.get("servoClaw");
        register();
        rotServo.getController().pwmEnable();
        servoClaw.getController().pwmEnable();

        isOpen = false;
    }
    public void periodic() {
        servoClaw.setPosition(isOpen?openClaw:closeClaw);
        dashboard.addData("gripperOpen: ", isOpen);
    }

    public Command setPickup(){
        return new InstantCommand(()->rotServo.setPosition(score));
    }
    public Command setScore(){
        return new InstantCommand(()->rotServo.setPosition(pickup));
    }

    public Command toggleClaw() {
        return new InstantCommand(()-> isOpen = !isOpen);
    }
}
