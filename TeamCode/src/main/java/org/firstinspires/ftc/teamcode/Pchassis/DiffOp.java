package org.firstinspires.ftc.teamcode.Pchassis;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class DiffOp extends OpMode {
    public Motor m0 = null, m1 = null, m2 = null, m3 = null;
    public MotorGroup lGroup, rGroup;
    public DifferentialDrive x;

    @Override
    public void init() {
        m0 = new Motor(hardwareMap, "m0");
        m1 = new Motor(hardwareMap, "m1");
        m2 = new Motor(hardwareMap, "m2");
        m3 = new Motor(hardwareMap, "m3");

        m0.set(0);
        m1.set(0);
        m2.set(0);
        m3.set(0);

        m0.setInverted(false);
        m1.setInverted(true);
        m2.setInverted(false);
        m3.setInverted(true);

        // Set motors to run with/without encoders
        m0.setRunMode(Motor.RunMode.VelocityControl);
        m1.setRunMode(Motor.RunMode.VelocityControl);
        m2.setRunMode(Motor.RunMode.VelocityControl);
        m3.setRunMode(Motor.RunMode.VelocityControl);


        m0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lGroup = new MotorGroup(m0,m2);
        rGroup = new MotorGroup(m1,m3);

        x = new DifferentialDrive(lGroup,rGroup);
    }

    @Override
    public void loop() {
        x.arcadeDrive(gamepad1.right_trigger - gamepad1.left_trigger,gamepad1.left_stick_x);
    }
}
