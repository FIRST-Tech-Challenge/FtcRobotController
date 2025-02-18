package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RollerIntakeBot extends DifferentialWristBot {
    private CRServo intakeRoller;
    private Servo colorIndicator;
    private NormalizedColorSensor colorSensor;

    float gain = 5;
    float thres = 50;

    final int ROLLER_MODE_OFF = 0;
    final int ROLLER_MODE_INTAKE = 1;
    final int ROLLER_MODE_OUTAKE = -1;
    final int NOTHING = 0;
    final int YELLOW = 1;
    final int BLUE = 2;
    final int RED = 3;
    int rollerMode = ROLLER_MODE_OFF;


    public RollerIntakeBot(LinearOpMode opMode)  {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        intakeRoller = ahwMap.get(CRServo.class, "intakeRoller");
        colorSensor = ahwMap.get(NormalizedColorSensor.class, "colorSensor");
        colorIndicator = ahwMap.get(Servo.class, "colorIndicator");

        colorSensor.setGain(gain);
    }

    public void setRollerPower(double power) {
//        intakeRoller.setPosition((power + 1) / 2); // Convert power (-1 to 1) to position (0 to 1)
        intakeRoller.setPower(power);
    }

    protected void onEvent(int event, int data) {
        super.onEvent(event, data);
        if (event == EVENT_SAMPLE_ROLLED_IN) {
            updateColorIndicator(data);
        }
        else if (event == EVENT_SAMPLE_ROLLED_OUT) {
            updateColorIndicator(NOTHING);
        }
    }

    protected void onTick() {
        super.onTick();

        telemetry.addData("Mode", rollerMode);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        @ColorInt int color = colors.toColor();
        telemetry.addData("Red", Color.red(color));
        telemetry.addData("Green", Color.green(color));
        telemetry.addData("Blue", Color.blue(color));
        telemetry.addData("Gain", gain);
        telemetry.addData("Thres", thres);

        int obj = getObjectInPlace();

        updateColorIndicator(obj);

        if (rollerMode == ROLLER_MODE_INTAKE) {
            telemetry.addData("Current obj", obj);
            if (obj != NOTHING) {
                stopRoller();
                triggerEvent(EVENT_SAMPLE_ROLLED_IN, obj);
            }
        } else if (rollerMode == ROLLER_MODE_OUTAKE) {
//            int obj = getObjectInPlace();
            if (obj == NOTHING) {
                stopRoller();
                triggerEvent(EVENT_SAMPLE_ROLLED_OUT, obj);
            }
        }
    }

    public void adjustGain(boolean increase) {
        if (increase) {
            gain += 0.003f;
        } else {
            gain = Math.max(1f, gain - 0.003f);
        }
        colorSensor.setGain(gain);
        telemetry.addLine()
                .addData("ColorSensor Gain", gain);
    }

    public void adjustThres(boolean increase) {
        if (increase) {
            thres += 0.1f;
        } else {
            thres -= 0.1f;
        }
//        colorSensor.setGain(gain);
        telemetry.addLine()
                .addData("Threshold", thres);
    }

    public void logColorSensor() {
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

    public void intake(boolean input) {
        if(input) {
            rollerMode = ROLLER_MODE_INTAKE;
            setRollerPower(-1); // Full power intake
        }
    }



    public void outake(boolean input) {
        if(input) {
            rollerMode = ROLLER_MODE_OUTAKE;
            setRollerPower(1); // Full power outake
        }
    }

    public void stopRoller() {
        rollerMode = ROLLER_MODE_OFF;

        // Gradually reduce power before stopping

        setRollerPower(1);   // Briefly reverse
        opMode.sleep(40);

        setRollerPower(0);     // Fully stop
    }

    public void setColorIndicator(double pos) {
        colorIndicator.setPosition(pos);
        telemetry.addData("Current Color: ", "%.3f", pos);
    }


    int getColor(int red, int green, int blue) {
        // 0 = nothing, 1 = yellow, 2 = blue, 3 = red

//        int thres = 100; // Adjust as needed based on lighting conditions
        //thres = 50, gain = 5
        //red: 140, 45, 45
        //yellow: 228, 228, 86
        //blue: 32, 70, 120

        if (red > 150 && green > 150) {
            return YELLOW; // Yellow (high red + green, low blue)
        } else if (red > 100) {
            return RED; // Red (high red, low green + blue)
        }  else if (blue > 100) {
            return BLUE; // Blue (high blue, low red + green)
        } else {
            return NOTHING; // No object detected
        }

    }
    public void updateColorIndicator(int curColor) {
        if (curColor == RED) { // red
            colorIndicator.setPosition(0.279);
        } else if (curColor == BLUE) { // blue
            colorIndicator.setPosition(0.611);
        } else if (curColor == YELLOW) { // yellow
            colorIndicator.setPosition(0.388);
        } else {
            colorIndicator.setPosition(0); // arbitrary value for nothing (green)
        }
    }
    public int getObjectInPlace() {
        // 0 = nothing, 1 = yellow, 2 = blue, 3 = red
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        @ColorInt int color = colors.toColor();
//
//        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 6.0) {
//            return 0;
//        }
        return getColor(Color.red(color), Color.green(color), Color.blue(color));
    }
}

