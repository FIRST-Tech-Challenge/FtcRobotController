package auto;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VSlide extends MainTest {

    private DcMotor motor;
    public VSlide(DcMotor motor) {
        this.motor = motor;
    }

    public void goToPosition(int position) {
        // Code to move V-Slide to specified position
    }

    @Override
    public void resetPosition() {
        // Code to reset V-Slide position using touch sensor
    }

    public void goToPosition(double position) {
        // Code to move V-Slide to specified position
    }
}