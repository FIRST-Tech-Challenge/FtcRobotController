package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashMap;
import java.util.HashSet;

public class ExtendableArm extends Arm {
    private final DcMotor ROTATION_MOTOR;

    public ExtendableArm(DcMotor rotationMotor, DcMotor extensionMotor, Servo clawRollServo, Servo clawYawServo, Servo clawGripServo) {
        this.ROTATION_MOTOR = rotationMotor;
        

        super.motors.add();
        super.motors.add(extensionMotor);
        super.servos.add(clawRollServo);
        super.servos.add(clawYawServo);
        super.servos.add(clawGripServo);
    }
}