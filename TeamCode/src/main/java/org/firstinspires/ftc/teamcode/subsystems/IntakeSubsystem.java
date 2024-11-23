package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private CRServo intakeServo;    
    private Servo wristServo;

    private HardwareMap hardwareMap;

    public static enum WristPositions {
        INTAKE_POSITION(0.35), //Check this value!!!
        OUTTAKE_POSITION(0); //Check this value!!!

        public double value;

        private WristPositions(double value) {
            this.value = value;
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);
        wristServo = hardwareMap.get(Servo.class, Constants.IntakeConstants.INTAKE_WRIST_SERVO_NAME);
    }

    public void intake() {
        intakeServo.setPower(1.0); //Check this value!
    }

    public void outtake() {
        intakeServo.setPower(-1.0); //Check this value! -> should be the opposite as the last ;)
    }

    public void stopIntakeServo() {
        intakeServo.setPower(0);
    }

    public void rotateToPosition(WristPositions position) {
        wristServo.rotateBy(position.value);
    }
}