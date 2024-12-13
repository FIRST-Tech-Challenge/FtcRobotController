package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

public class IntakeSubsystem extends SubsystemBase {

    public static final float INTAKE_SPEED = .95f;

    DcMotorEx powerMotor;
    ServoEx tilterServo;
    boolean currentState = false;
    private final static FTCDashboardPackets dbp = new FTCDashboardPackets("IntakeSubsystem");

    public IntakeSubsystem(DcMotorEx powerMotor, ServoEx tilterServo) {
        this.powerMotor = powerMotor;
        this.tilterServo = tilterServo;
        tilterServo.setRange(0, 45);
        setIntakeDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void tiltIntake() {
        tilterServo.setPosition(1);
        dbp.info("Tilting Intake (Down)");
        dbp.send(true);
    }

    public void untiltIntake() {
        tilterServo.setPosition(0);
        dbp.info("Untilting Intake (Up)");
        dbp.send(true);
    }

    public void setIntakeState(boolean activated) {
        powerMotor.setPower(activated ? INTAKE_SPEED : .05f);
        currentState = activated;
        dbp.info("Setting Intake Activation State: "+activated);
        dbp.send(true);
    }

    public void setIntakeDirection(DcMotorSimple.Direction direction) {
        powerMotor.setDirection(direction);
    }

    public void toggleIntakeState() {
        setIntakeState(!currentState);
    }

}
