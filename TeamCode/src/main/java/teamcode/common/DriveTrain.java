package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface DriveTrain {

    void moveToPosition(double desiredPosition, double desiredVelocity);

    void moveToRotation(double desiredRotation, double omega);

    void setPower(double power);

    DcMotor[] getMotors();


}
