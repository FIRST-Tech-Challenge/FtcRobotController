
package auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HSlide extends MainTest{

    private DcMotor motor;

    public HSlide(DcMotor motor) {
        // Constructor code if needed
        this.motor = motor;
    }

    public void resetPosition() {
        // Code to reset H-Slide position using touch sensor
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void goToPosition(int position) {
        // Code to move H-Slide to specified position
        motor.setTargetPosition(position);
    }

}