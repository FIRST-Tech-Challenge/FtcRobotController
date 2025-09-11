package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartIntake {
    private final DcMotor leftIntake;
    private final DcMotor rightIntake;
    private final Gamepad gamepad;
    private boolean sorting = false;
    private boolean buttonPressed = false;
    public SmartIntake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        gamepad = hardwareMap.get(Gamepad.class, "gamepad1");
    }
    public void intake(boolean swap_sort) {
        if (swap_sort && !buttonPressed) {
            sorting = !sorting;
            buttonPressed = true;
        } else if (!swap_sort && buttonPressed) {
            buttonPressed = false;
        }
        if (sorting) {

        } else {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
    }

}
