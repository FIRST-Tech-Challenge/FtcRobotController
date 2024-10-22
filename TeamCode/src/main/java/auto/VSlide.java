package auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class VSlide extends MainTest {

    private DcMotor motor;
    public VSlide(DcMotor motor) {
        this.motor = motor;
    }

    public void goToPosition(int position) {
        // Code to move V-Slide to specified position
        motor.setTargetPosition(position);
    }

    @Override
    public void resetPosition() {
        // Code to reset V-Slide position using touch sensor
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToPosition(double position) {
        // Code to move V-Slide to specified position
        motor.setTargetPosition((int) position);
    }
}