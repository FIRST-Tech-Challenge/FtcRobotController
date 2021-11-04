package org.firstinspires.ftc.teamcode.Pchassis;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.*;
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

        // reset the encoders
        robot.m0.resetEncoder();
        robot.m1.resetEncoder();
        robot.m2.resetEncoder();
        robot.m3.resetEncoder();

        // grab the encoder instances
        Motor.Encoder encoder0 = robot.m0.encoder;
        Motor.Encoder encoder1 = robot.m0.encoder;
        Motor.Encoder encoder2 = robot.m0.encoder;
        Motor.Encoder encoder3 = robot.m0.encoder;

        // get number of revolutions for each encoder
        double revolutions0 = encoder0.getRevolutions();
        double revolutions1 = encoder1.getRevolutions();
        double revolutions2 = encoder2.getRevolutions();
        double revolutions3 = encoder3.getRevolutions();

        // set the distance per pulse to 18 mm / tick
        encoder0.setDistancePerPulse(18.0);
        encoder1.setDistancePerPulse(18.0);
        encoder2.setDistancePerPulse(18.0);
        encoder3.setDistancePerPulse(18.0);

        // get the distance traveled
        double distance0 = encoder0.getDistance();
        double distance1 = encoder1.getDistance();
        double distance2 = encoder2.getDistance();
        double distance3 = encoder3.getDistance();
    }
}