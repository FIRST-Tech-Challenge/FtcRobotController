package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.rustlib.commandsystem.Subsystem;
import org.rustlib.rustboard.Rustboard;

public class Climber extends Subsystem {
    public final DcMotor motor;
    public final Servo servo;

    public Climber(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "climbMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(Servo.class, "climbServo");
    }

    public void deliverHook() {
        servo.setPosition(Rustboard.getDouble("climb up", 0.07));
    }

    public void hookDown() {
        servo.setPosition(Rustboard.getDouble("climb down", 0.45));
    }

    public void winch(double speed) {
        motor.setPower(speed);
    }
}
