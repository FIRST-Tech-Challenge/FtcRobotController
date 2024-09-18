package org.firstinspires.ftc.teamcode.hardware;

public class CraneSystem extends ArmSystem {
    public CraneSystem(String[] motorNames) {
        motors = new DcMotor[motorNames.length];

        for (int i=0; i<motorNames.length; i++) {
            motors[i] = new DcMotor(DcMotor.class, motorNames[i]);
        }
    }
}