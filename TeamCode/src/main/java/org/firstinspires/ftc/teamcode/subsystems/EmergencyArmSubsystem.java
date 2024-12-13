package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.Objects;

public class EmergencyArmSubsystem extends SubsystemBase {

    public DcMotorEx armMotorLower, armMotorHigher;
    public ServoEx pinchServo;
    public CRServo wristServo;

    public static final double THRESHOLD = .2f;
    public static final double SPEED = .75f;

    public EmergencyArmSubsystem(HardwareMap map) throws Exception {
        armMotorLower = RobotHardwareInitializer.MotorComponent.LOWER_ARM.getEx(map);
        armMotorHigher = RobotHardwareInitializer.MotorComponent.HIGHER_ARM.getEx(map);
        pinchServo = RobotHardwareInitializer.ServoComponent.PINCHER.getEx(map);
        wristServo = RobotHardwareInitializer.CRServoComponent.WRIST.get(map);
    }

    /**
     * @param power between [-1, 1]
     */
    public void setLowerArmPower(double power) {
        armMotorLower.setPower(power * SPEED);
        if (Math.abs(power) < THRESHOLD) { armMotorLower.setPower(0); }
    }

    /**
     * @param power between [-1, 1]
     */
    public void setHigherArmPower(double power) {
        armMotorHigher.setPower(power * SPEED);
        if (Math.abs(power) < THRESHOLD) { armMotorHigher.setPower(0); }
    }

    public enum PinchState {
        PINCHED(0), // left state
        OPEN(0.5f), // middle state
        ;

        public final double pinchPosition;
        PinchState(double pinchPosition) {
            this.pinchPosition = pinchPosition;
        }
    }

    public void setPinchState(PinchState state) {
        Objects.requireNonNull(state);
        pinchServo.setPosition(state.pinchPosition);
    }

    /**
     * @param power between [-1, 1]
     */
    public void setWristPower(double power) {
        wristServo.setPower(power * SPEED);
        if (Math.abs(power) < THRESHOLD) { wristServo.setPower(0); }
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
