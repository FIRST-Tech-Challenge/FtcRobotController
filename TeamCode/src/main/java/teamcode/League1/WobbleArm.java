package teamcode.League1;

import android.view.animation.RotateAnimation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm {
    // is attached to the back of the robot as a rigid arm that rotates at the base to lift
    private static final double ROTATION_VAL_Raise = 1;
    private static final double ROTATIONAL_VAL_Lower = 0;

    Servo armServo;

    public WobbleArm(HardwareMap hardwareMap) {
        // constructor

        // single servo being used for arm rotation ( 10/7/20 )
        armServo = hardwareMap.servo.get("WobbleGrabber");
    }

    public void raise() {

        armServo.setPosition(ROTATION_VAL_Raise);
    }

    public void lower() {

        armServo.setPosition(ROTATIONAL_VAL_Lower);
    }

}
