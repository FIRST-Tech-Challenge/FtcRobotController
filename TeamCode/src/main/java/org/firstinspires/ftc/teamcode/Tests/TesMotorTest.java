package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class TesMotorTest extends RFMotorTest {
    public static double RESISTANCE, kS, kV, kA, MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD;
    public void runOpMode() throws InterruptedException {
        initialize("leftLiftMotor", 3000,0, DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            setConstants(RESISTANCE, kS, kV,kA,MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD);
            auto();
        }
        stop();
    }
}
