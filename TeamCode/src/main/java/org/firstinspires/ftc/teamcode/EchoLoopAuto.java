package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robot_utilities.FlyWheel;

@Autonomous(name="EchoLoopAuto")
public class EchoLoopAuto extends LinearOpMode {
    private Motor driveLeft, driveRight;
    private FlyWheel flywheel;
    private Servo hitter;
    private Motor intake1, intake2;

    private ElapsedTime elapsedTime;


    public void initRobot() {

        elapsedTime = new ElapsedTime();

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE));
    }
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        elapsedTime.reset();

        double speed = 0.7;
        while(elapsedTime.seconds() < 2) {
            driveLeft.set(speed);
            driveRight.set(-speed);
        }

        stop();
    }
}
