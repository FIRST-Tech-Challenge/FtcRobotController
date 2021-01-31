package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This class tests the average rotational speed (in clicks/ms) of the drive motors at different power levels in software, from 0.1 to 1.2.

@Autonomous(group="ChrisBot", name="JACK UP ROBOT - Encoder Motor Speed Test")

public class encoderMotorSpeedTest extends LinearOpMode {

    public int[] range(int start, int end) {
        if (end - start <= 0) {
            return new int[]{};
        }
        else {
            int[] arr = new int[end - start];
            for(int i = start; i < end; i++) {
                arr[i-start] = i;
            }
            return arr;
        }
    }

    chrisBot robot = new chrisBot();

    final long dt = 2000;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        waitForStart();

        telemetry.addLine("**********************************\nStarted");
        telemetry.update();

        Telemetry.Item motorSpeed = telemetry.addData("current motor speed",0);

        double speed = 0;

        int flPos, frPos, blPos, brPos;
        int flD, frD, blD, brD;

        for(int i : range(1,13)) {
            if(!opModeIsActive()) {
                break;
            }
            if(robot.intakeExists) {
                robot.motorIntake.setPower(1);
            }
            if(robot.shooterExists) {
                robot.motorShooter1.setPower(1);
            }
            flPos = robot.motorFrontLeft.getCurrentPosition();
            frPos = robot.motorFrontRight.getCurrentPosition();
            blPos = robot.motorBackLeft.getCurrentPosition();
            brPos = robot.motorBackRight.getCurrentPosition();

            speed = (double)(i)/10;
            motorSpeed.setValue(speed);
            telemetry.update();
            robot.setAllPower(speed);

            sleep(dt);

            flD = robot.motorFrontLeft.getCurrentPosition() - flPos;
            frD = robot.motorFrontRight.getCurrentPosition() - frPos;
            blD = robot.motorBackLeft.getCurrentPosition() - blPos;
            brD = robot.motorBackRight.getCurrentPosition() - brPos;

            telemetry.addData("FL Average motor clicks/ms at speed"+ speed, (double)flD/dt);
            telemetry.addData("FR Average motor clicks/ms at speed"+ speed, (double)frD/dt);
            telemetry.addData("BL Average motor clicks/ms at speed"+ speed, (double)blD/dt);
            telemetry.addData("BR Average motor clicks/ms at speed"+ speed, (double)brD/dt);
            telemetry.update();
        }

        robot.setAllPower(0);

        if(robot.intakeExists) {
            robot.motorIntake.setPower(0);
        }
        if(robot.shooterExists) {
            robot.motorShooter1.setPower(0);
        }

        telemetry.addLine("Test done");
        telemetry.update();

        sleep(60000);

    }
}
