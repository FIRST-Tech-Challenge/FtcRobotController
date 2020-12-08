package org.firstinspires.ftc.samplecode.strafer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.logger.Log;
import com.technototes.logger.Loggable;

public class Hardware implements Loggable {
    //drive motors
    @Log.Number(name = "flmotor")
    public Motor<DcMotor> flMotor;
    @Log.Number(name = "frmotor")
    public Motor<DcMotor> frMotor;
    @Log.Number(name = "rlmotor")
    public Motor<DcMotor> rlMotor;
    @Log.Number(name = "rrmotor")
    public Motor<DcMotor> rrMotor;
    // hellooooooo
    public IMU imu;

    //new stuff
    public RangeSensor rangeSensor;


    public Hardware(){
        flMotor = new Motor<>("fl");
        frMotor = new Motor<>("fr");
        rlMotor = new Motor<>("rl");
        rrMotor = new Motor<>("rr");

        imu = new IMU("imu1");


        rangeSensor = new RangeSensor("range");
    }

}
