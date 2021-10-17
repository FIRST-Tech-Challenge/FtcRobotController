package org.firstinspires.ftc.teamcode.tools;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private final DcMotor liftMotor;
    private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private final GamepadEx stick;
    private final double afloatValue = 0.05;
    public void update() {
        if (!topSensor.getState()) {
            double stickValue = stick.getLeftY();
            if (stickValue >= 0.05 || stickValue <= -0.05) {
                liftMotor.setPower(stickValue);
            } else if (bottomSensor.getState()) {
                liftMotor.setPower(0);
            } else {
                liftMotor.setPower(afloatValue);
            }
        } else {
            liftMotor.setPower(afloatValue);
        }
    }

    /**
     *
     * @param map local hardwareMap instance
     * @param toolGamepad instance of FtcLib GamepadX
     */
    public Lift(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        this.liftMotor = map.get(DcMotor.class,"liftMotor");
        this.bottomSensor = map.get(DigitalChannel.class,"bottomSensor");
        this.topSensor = map.get(DigitalChannel.class,"topSensor");
        this.stick = toolGamepad;
    }
}
