package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;
    private final Servo wristServo;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final OpMode opMode;

    public static enum WristPositions {
        INTAKE_POSITION(Constants.IntakeConstants.WRIST_INTAKE_POS), //Check this value!!!
        OUTTAKE_POSITION(Constants.IntakeConstants.WRIST_OUTTAKE_POS); //Check this value!!!

        public double value;

        private WristPositions(double value) {
            this.value = value;
        }
    }

    public IntakeSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

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
        wristServo.setPosition(position.value);
    }

    public double getCurrentPosition() {
        return wristServo.getPosition();
    }
}