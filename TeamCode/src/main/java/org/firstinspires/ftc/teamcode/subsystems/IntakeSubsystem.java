package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

public class IntakeSubsystem extends SubsystemBase {

    DcMotorEx powerMotor;
    ServoEx tilterServo;
    boolean currentState = false;
    private final static FTCDashboardPackets dbp = new FTCDashboardPackets("IntakeSubsystem");

    public IntakeSubsystem(DcMotorEx powerMotor, ServoEx tilterServo) {
        this.powerMotor = powerMotor;
        this.tilterServo = tilterServo;
        tilterServo.setRange(0, 45);
    }

    public void tiltIntake() {
        tilterServo.setPosition(1);
        dbp.info("Tilting Intake");
        dbp.send(true);
    }

    public void untiltIntake() {
        tilterServo.setPosition(0);
        dbp.info("Untilting Intake");
        dbp.send(true);
    }

    public void setIntakeState(boolean activated) {
        powerMotor.setPower(activated ? 1 : 0);
        currentState = activated;
        dbp.info("Setting Intake Activation State: "+activated);
        dbp.send(true);
    }

    public void toggleIntakeState() {
        setIntakeState(!currentState);
    }

}
