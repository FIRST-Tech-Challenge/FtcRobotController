package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RollerIntakeBot extends FourWheelDriveBot {
    private Servo intakeRoller;
    private Servo colorIndicator;
    private NormalizedColorSensor colorSensor;

    float gain = 2;

    final int ROLLER_MODE_OFF = 0;
    final int ROLLER_MODE_INTAKE = 1;
    final int ROLLER_MODE_OUTAKE = -1;
    int rollerMode = ROLLER_MODE_OFF;


    public RollerIntakeBot(LinearOpMode opMode)  {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        intakeRoller = ahwMap.get(Servo.class, "intakeRoller");
        colorSensor = ahwMap.get(NormalizedColorSensor.class, "colorSensor");
        colorIndicator = ahwMap.get(Servo.class, "colorIndicator");
    }

    public void setRollerPower(double power) {
        intakeRoller.setPosition((power + 1) / 2); // Convert power (-1 to 1) to position (0 to 1)
    }

    protected void onEvent(int event, int data) {
        super.onEvent(event, data);
        if (event == EVENT_SAMPLE_ROLLED_IN) {
            updateColorIndicator();
        }

    }

    protected void onTick() {
        super.onTick();

        if (rollerMode == ROLLER_MODE_INTAKE) {
            if (isObjectInPlace()) {
                stopRoller();
                int color = 25555493;
                triggerEvent(EVENT_SAMPLE_ROLLED_IN, color);
            }
        } else if (rollerMode == ROLLER_MODE_OUTAKE) {
            if (!isObjectInPlace()) {
                stopRoller();
            }
        }
    }

    public void adjustGain(boolean increase, Telemetry telemetry) {
        if (increase) {
            gain += 0.003f;
        } else {
            gain = Math.max(1f, gain - 0.003f);
        }
        colorSensor.setGain(gain);
        telemetry.addLine()
                .addData("ColorSensor Gain", gain);
    }

    public void logColorSensor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
    }

    public void intake() {
        rollerMode = ROLLER_MODE_INTAKE;
        setRollerPower(1.0); // Full power intake
    }

    public void outake() {
        rollerMode = ROLLER_MODE_OUTAKE;
        setRollerPower(-1.0); // Full power outake
    }

    public void stopRoller() {
        rollerMode = ROLLER_MODE_OFF;
        setRollerPower(0.0); // Stop the roller
    }
    public void updateColorIndicator() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // convert the colors from getNormalizedColors into RGB int values from the toColor function
        @ColorInt int color = colors.toColor();
        // make the values from color variable into separate red, green, and blue values
        int red = Color.red(color);
        int green = Color.green(color);
        int blue = Color.blue(color);
        if (red > 200 && green < 70 && blue < 50) { // red
            colorIndicator.setPosition(-1.0); // arbitrary value for red block
        } else if (red < 50 && green < 70 && blue > 200) { // blue
            colorIndicator.setPosition(0.0); // arbitrary value for blue block
        } else if (red > 180 && green > 180 && blue < 50) { // yellow
            colorIndicator.setPosition(1.0); // arbitrary value for yellow block
        } else {
            colorIndicator.setPosition(0.5); // arbitrary value for nothing
        }
    }
    public boolean isObjectInPlace() {
        // TODO : add the real condition to detect the object
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        @ColorInt int color = colors.toColor();
        // Normalize colors, taking into consideration gain
        int red = Color.red(color);
        int green = Color.green(color);
        int blue = Color.blue(color);

        return (red > 180 || green > 180 || blue > 200) && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 6.0;
    } // Arbitrary values for ratio conditions, and distance sensor value

}