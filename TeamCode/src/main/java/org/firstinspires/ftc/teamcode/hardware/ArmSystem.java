package firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public abstract class ArmSystem {
    protected DcMotor[] armMotors;

    public ArmSystem(DcMotor[] armMotors) {
        this.armMotors = armMotors;
    }
}