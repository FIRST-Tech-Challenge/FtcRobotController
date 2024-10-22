package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class SliderTest extends OpMode {

    private DcMotor sliderMotor;
    private static final double SLIDER_UP_POWER = 0.5;
    private static final double SLIDER_DOWN_POWER = -0.5;

    @Override
    public void init() {
        // Initialize the slider motor
        sliderMotor = hardwareMap.get(DcMotor.class, "slider_motor");
    }

    @Override
    public void loop() {
        // Control the slider motor with gamepad buttons
        if (gamepad1.dpad_up) {
            sliderMotor.setPower(SLIDER_UP_POWER);
        } else if (gamepad1.dpad_down) {
            sliderMotor.setPower(SLIDER_DOWN_POWER);
        } else {
            sliderMotor.setPower(0);
        }

        // Send telemetry message to signify slider motor power
        telemetry.addData("Slider Power", sliderMotor.getPower());
    }
}