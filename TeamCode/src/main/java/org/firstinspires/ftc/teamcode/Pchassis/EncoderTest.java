package org.firstinspires.ftc.teamcode.Pchassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder Test", group="Pushbot")
public class EncoderTest extends LinearOpMode {
    public static final double dPower = 0.75;

    Hardware robot = new Hardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.m.driveRobotCentric(0,0,0);

// reset the encoder
        robot.m0.resetEncoder();

//// get the current velocity
//        double velocity = m_motor.getVelocity(); // only for MotorEx
//        double corrected = m_motor.getCorrectedVelocity();

// grab the encoder instance
        Motor.Encoder encoder = m_motor.encoder;

// get number of revolutions
        double revolutions = encoder.getRevolutions();

// set the distance per pulse to 18 mm / tick
        encoder.setDistancePerPulse(18.0);
        m_motor.setDistancePerPulse(18.0); // also an option

// get the distance traveled
        double distance = encoder.getDistance();
        distance = m_motor.getDistance(); // also an option

/** USEFUL FEATURE **/

// you can set the encoder of the motor to a different motor's
// encoder
        m_motor.encoder = other_motor.encoder;

    }
}