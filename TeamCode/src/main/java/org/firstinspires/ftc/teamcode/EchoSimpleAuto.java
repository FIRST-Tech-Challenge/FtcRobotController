package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="EchoSimpleAuto")
public class EchoSimpleAuto extends LinearOpMode {
    private Motor driveLeft, driveRight;
    public void initRobot() {


        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();
        double speed = 0.7;
        driveLeft.set(-speed);
        driveRight.set(speed);

        Thread.sleep(2000);

        stop();
    }
}
