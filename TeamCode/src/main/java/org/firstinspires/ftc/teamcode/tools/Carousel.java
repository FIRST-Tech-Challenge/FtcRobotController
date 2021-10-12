package org.firstinspires.ftc.teamcode.tools;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.NonNull;

public class Carousel {
    private final CRServo spinner;
    private final ToggleButtonReader bReader;
    public void update() {
        bReader.readValue();
        if (bReader.getState()) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }
    }
    public Carousel(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        this.spinner = map.get(CRServo.class,"Spinner");
        this.bReader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.B);
    }
}
