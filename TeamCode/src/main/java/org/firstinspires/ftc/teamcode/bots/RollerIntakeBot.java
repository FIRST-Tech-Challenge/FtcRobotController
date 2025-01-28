package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;

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
    }

    public void setRollerPower(double power) {
        intakeRoller.setPosition((power + 1) / 2); // Convert power (-1 to 1) to position (0 to 1)
    }

    protected void onTick() {
        super.onTick();
        if (rollerMode == ROLLER_MODE_INTAKE) {
            if (isObjectInPlace()) {
                stopRoller();
            }
        } else if (rollerMode == ROLLER_MODE_OUTAKE) {
            if (!isObjectInPlace()) {
                stopRoller();
            }
        }
    }

    public void adjustGain(boolean increase, Telemetry telemetry) {
        if (increase) {
            gain += 0.005;
        } else {
            gain -= 0.005;
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

    public boolean isObjectInPlace() {
        // TODO : add the real condition to detect the object
        return colorSensor.getNormalizedColors().red > 0.5;
    }
}