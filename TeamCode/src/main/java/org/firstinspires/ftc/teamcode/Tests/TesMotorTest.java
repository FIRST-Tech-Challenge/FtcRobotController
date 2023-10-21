package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
@Config
@Autonomous(name = "liftMotorTest")
public class TesMotorTest extends RFMotorTest {
    public static double max = 3000, min =0 , RESISTANCE = 0, kS = 0.04,kV =  3.2786E-4,kA= 6E-5,
            MAX_UP_VELO= 3000, MAX_DOWN_VELO=-2280,MAX_ACCEL =  10000, MAX_DECEL = -66974,kP= 0,kD= 0;
    public void runOpMode() throws InterruptedException {
        initialize("motorRightFront",(int)max,(int)min, DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            setConstants(max, min , RESISTANCE, kS, kV,kA,MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD);
            auto();
        }
        stop();
    }
}
