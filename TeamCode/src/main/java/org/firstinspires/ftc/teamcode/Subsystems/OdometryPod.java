package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;


import org.firstinspires.ftc.teamcode.RobotContainer;




/** Subsystem */
public class OdometryPod extends SubsystemBase {

    // Local objects and variables here

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.024; // m
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 0.315; // m; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0.100; // m; offset of the lateral wheel

    private MotorEx leftEncoderPod;
    private MotorEx rightEncoderPod;
    private MotorEx frontEncoderPod;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    /** Place code here to initialize subsystem */
    public OdometryPod() {

        leftEncoderPod = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap,"leftEncoder");
        rightEncoderPod = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap, "rightEncoder");
        frontEncoderPod = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap, "frontEncoder");

        leftEncoderPod.setInverted(true);

        leftEncoder = leftEncoderPod.encoder;
        rightEncoder = rightEncoderPod.encoder;
        frontEncoder = frontEncoderPod.encoder;

        leftEncoder.setDistancePerPulse(Math.PI * 2.0 * WHEEL_RADIUS/(TICKS_PER_REV*GEAR_RATIO));
        rightEncoder.setDistancePerPulse(Math.PI * 2.0 * WHEEL_RADIUS/(TICKS_PER_REV*GEAR_RATIO));
        frontEncoder.setDistancePerPulse(Math.PI * 2.0 * WHEEL_RADIUS/(TICKS_PER_REV*GEAR_RATIO));

        leftEncoder.reset();
        rightEncoder.reset();
        frontEncoder.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        RobotContainer.ActiveOpMode.telemetry.addData("leftEncoder",leftEncoder.getDistance());
        RobotContainer.ActiveOpMode.telemetry.addData("rightEncoder",rightEncoder.getDistance());
        RobotContainer.ActiveOpMode.telemetry.addData("frontEncoder",frontEncoder.getDistance());

        RobotContainer.ActiveOpMode.telemetry.update();
    }

    // place special subsystem methods here

    public double getLeftEncoderDistance() {
        return leftEncoder.getDistance();
    }
    public double getRightEncoderDistance() {
        return rightEncoder.getDistance();
    }
    public double getFrontEncoderDistance() {
        return frontEncoder.getDistance();
    }
}