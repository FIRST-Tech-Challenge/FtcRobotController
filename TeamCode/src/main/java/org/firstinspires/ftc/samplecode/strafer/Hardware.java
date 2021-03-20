package org.firstinspires.ftc.samplecode.strafer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.logger.Log;
import com.technototes.logger.Loggable;

public class Hardware implements Loggable {
    //drive motors
    @Log.Number(name = "flmotor")
    public EncodedMotor<DcMotor> flMotor;
    @Log.Number(name = "frmotor")
    public EncodedMotor<DcMotor> frMotor;
    @Log.Number(name = "rlmotor")
    public EncodedMotor<DcMotor> rlMotor;
    @Log.Number(name = "rrmotor")
    public EncodedMotor<DcMotor> rrMotor;
    // hellooooooo
    public IMU imu;


    public Hardware(){
        flMotor = new EncodedMotor<>("fl");
        frMotor = new EncodedMotor<>("fr");
        rlMotor = new EncodedMotor<>("rl");
        rrMotor = new EncodedMotor<>("rr");

        imu = new IMU("imu1");
        
    }

}
