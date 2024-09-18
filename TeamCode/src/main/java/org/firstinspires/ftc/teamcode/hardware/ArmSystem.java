package firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public abstract class ArmSystem {
    protected DcMotor[] motors;

    public ArmSystem(DcMotor[] motors) {
        this.motors = motors;
    }

    public ArmSystem(String[] motorNames) {
        motors = new DcMotor[motorNames.length];

        for (int i=0; i<motorNames.length; i++) {
            motors[i] = new DcMotor(DcMotor.class, motorNames[i]);
        }
    }
}