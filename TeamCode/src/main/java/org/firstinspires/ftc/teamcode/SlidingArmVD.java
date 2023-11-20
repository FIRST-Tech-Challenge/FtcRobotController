package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.ConfigMan;

import java.util.HashMap;

public class SlidingArmVD extends VirtualDevice implements ConfigMan {
    protected double rotationalPosition;
    protected int lengthPosition;
    public double power;
    private DcMotorEx motor;
    private HashMap<String, String> configMan;

    public void restConfiguration() {

    }


    public SlidingArmVD(Manufacturer manufacturer, String partNumber, String DeviceName, HashMap<String, Integer> connectionPortScheme, int version,DcMotorEx drivingMotor) {
        super( partNumber, DeviceName, connectionPortScheme, version);
        motor = drivingMotor;
    }
    public SlidingArmVD( String partNumber, String DeviceName, HashMap<String, Integer> connectionPortScheme, int version,DcMotorEx drivingMotor) {
        super(partNumber, DeviceName, connectionPortScheme, version);
        motor = drivingMotor;
    }
    public SlidingArmVD(Manufacturer manufacturer, String partNumber, String DeviceName, HashMap<String, Integer> connectionPortScheme, int version,DcMotorEx drivingMotor, HashMap<String,String> configurationOptions) {
        super( partNumber, DeviceName, connectionPortScheme, version);
        motor = drivingMotor;
        configMan = configurationOptions;

    }

    @Override
    public void close() {
    }
    public void SetPosition(double newPosition) {
        // Rotational Position
        motor.setPower(power);
        motor.setTargetPosition((int) newPosition);
    }
    public void SetPosition(double newPosition, double time) {
        power = Range.clip(((newPosition-rotationalPosition)/time/DriveConstants.MAX_RPM),0,1);
        motor.setPower(power);
        motor.setTargetPosition((int) newPosition);
    }
    public void SetPosition(double newPosition, float speed) {
        power = Range.clip(speed/ DriveConstants.MAX_RPM,0,1);
        motor.setPower(power);
        motor.setTargetPosition(Range.clip((int) newPosition, 0, 2080));
    }
    public void SetPosition(int newPosition) {
        motor.setTargetPosition((int) newPosition);
    }
    public double getRotationalPosition() {
        return rotationalPosition;
    }
    public int getLengthPosition() {
        return lengthPosition;
    }
    /** Moves at the speed of the system. */
    public void moveAtSpeed(double angularSpeed) {
        motor.setVelocity(angularSpeed);
    }
    public void moveAtSpeed(double angularSpeed, BNO055IMU.AngleUnit unit) {
        motor.setVelocity(angularSpeed, unit.toAngleUnit());
    }
    public void moveAtSpeed(float lengthSpeed) {
        motor.setVelocity(lengthSpeed / 120 ); // 120 is the MM per ROTATIONS. I'm hoping that this'll work.
    }

    /** Default values for the config file. */

    @Override
    public void resetConfiguration() {
        configMan.put("MotorTickCount", "384.5");
        configMan.put("MotorMaxRPM", "435");
    }
    @Override
    public void setConfig(HashMap<String, String> config) {
        configMan = config;
    }

    @Override
    public String getConfig(String key) {
        return configMan.get(key);
    }
    @Override
    public String addConfigData(String key, String data) {
        configMan.put(key,data);
        return data;
    }

    @Override
    public void replaceConfigKey(String key, String data) throws Exception {
        configMan.put(key,data);
        if (configMan.get(key) != null) {
            throw new Exception("Value not found in Hash.");
        }
    }
    public void runWithController(double input, ElapsedTime updateDelta) {
        if (this.getRotationalPosition() + (input * updateDelta.time() * 435) < 5088) {
            this.SetPosition((int) getRotationalPosition() + (input + updateDelta.time() * 435));
        }
    }
}
