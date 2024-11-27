package com.kalipsorobotics.actions.intake;

import static org.firstinspires.ftc.robotcore.external.JavaUtil.colorToHue;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeNoodleAction {

    Intake intake;

    DcMotor intakeNoodleMotor;
    ColorSensor colorSensor;

    public IntakeNoodleAction(Intake intake) {
        this.intake = intake;
        this.intakeNoodleMotor = intake.getNoodleMotor();
        this.colorSensor = intake.getColorSensor();
        colorSensor.enableLed(false);
    }


    public void run() {
        intakeNoodleMotor.setPower(1);
    }


    public boolean colorSense(String color) {
        if (color.equalsIgnoreCase("blue")) {
            return colorSensor.blue() >= colorSensor.red();
        } else return false;
    }

    /*
    YELLOW: R:2372 G:4803 B:5166
    BLUE: same as yellow
    RED:
     */

    public double blue() {
        return colorToHue(colorSensor.blue());
    }
    public double red() {
        return colorToHue(colorSensor.red());
    }
    public double green() {
        return colorToHue(colorSensor.green());
    }
    public double average() {
        return colorSensor.argb();
    }
    public void reverse() {
        intakeNoodleMotor.setPower(-1);
    }
    public void stop() {
        intakeNoodleMotor.setPower(0);
    }
}
