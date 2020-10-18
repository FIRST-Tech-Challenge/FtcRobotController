package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TensorDrivingTest", group="Zippo")
//@Disabled

public class TensorDrivingTest extends OpMode {
    testPlatformHardware robot  = new testPlatformHardware();

    private String key = vuforia_key.key1;

    // Create variables for motor power
    private double flPower = 0; //left wheel
    private double frPower = 0; //right wheel
    private double blPower = 0; //front power
    private double brPower = 0; //back power

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        double time =  System.currentTimeMillis();

    }

    public void loop() {

    }
}
