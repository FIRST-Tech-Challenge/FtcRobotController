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
    public Servo servoClaw;
    Servo servo2;
    Servo rotServo;
    public Servo rotServo2;
    public boolean isOpen = false;
    public boolean isPickup;
    public GripperSubsystem(HardwareMap map){
        rotServo = map.servo.get("rotServo");
        rotServo2 = map.servo.get("servoRot");
        servoClaw = map.servo.get("servoClaw");
        register();
        rotServo.getController().pwmEnable();
        servoClaw.getController().pwmEnable();




    }
    public void periodic() {
        servoClaw.setPosition(isOpen?openClaw:closeClaw);
        dashboard.addData("gripperOpen: ", isOpen);
    }

    public Command setPickup(){
        return new InstantCommand(()->rotServo2.setPosition(pickup));
    }
    public Command setScore(){
        return new InstantCommand(()->rotServo2.setPosition(score));
    }

    public Command toggleClaw() {
        return new InstantCommand(()-> isOpen = !isOpen);
    }
    public Command openClaw() {
        return new InstantCommand(()-> isOpen = true);
    }
    public Command closeClaw() {
        return new InstantCommand(()-> isOpen = false);
    }


}
