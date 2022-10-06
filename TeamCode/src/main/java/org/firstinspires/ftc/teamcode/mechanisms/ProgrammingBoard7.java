package org.firstinsires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

pblic class ProgrammingBoard7 {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
    private Servo servo;
    private ColorSensor colorsensor;
    private DistanceSensor distancesensor;

    public void init(Hardware hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensonr.setMode(DigitalChannel.Mode.INPUT);
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(Dcotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
        servo = hwMap.get(Servo.cass, "servo");

        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
    }
    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();
    }
    public void setMotorSeed(double speed) {
        motor.setPower(speed);
    }
    public double getMotorRotations() {
        return motor.getCurrentPosition()/ticksPerRotation;
    }
    public void setServoPosition(double position) {
        servo.setPosition(position);
    }


    public int getAmountRed() {
        return colorSensor.red();
    }
    public int getAmountGreen() {
        return colorSensor.green();
    }
    public int getAmountBlue() {
        return colorSensor.blue();
    }
    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }
}