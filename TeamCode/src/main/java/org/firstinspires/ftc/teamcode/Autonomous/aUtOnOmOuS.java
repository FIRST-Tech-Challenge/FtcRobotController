package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Constants;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

import java.util.List;

@Autonomous(name="Auto-Main")

public class aUtOnOmOuS extends LinearOpMode {


    int step = 1;



    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry(); //AND THIS BEFORE COMPETITION also line 109
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        Input input = new Input(hardwareMap);
        Motors motors = new Motors(hardwareMap);
        IMU imu = new IMU(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            if (step == 1) {
                //drive to buckets
                if (!(input.getDistance() >= 14)) {
                    input.move(-10);
                }
                else {
                    input.move(0);
                    step++;
                }


            }
            if (step == 2) {
                //extend
                input.arm(1202);

                if (!(motors.getUpArmPosition() <= -8091)) {
                    input.upArm(50);
                }
                else {
                    input.upArm(0);
                    step++;
                }


            }
            if (step == 3) {
                //drop sample
                input.claw(false,true);

                step++;
            }
            if (step == 4) {
                //retract
                input.arm(0);

                if (!(motors.getUpArmPosition() >= 0)) {
                    input.upArm(50);
                }
                else {
                    input.upArm(0);
                    step++;
                }



            }
            if (step == 5) {
                //turn
                if (!((imu.getAngle('y') < -88) && (imu.getAngle('y') > -92))) {
                    input.spin(50);
                }
                else {
                    input.spin(0);
                    step++;
                }


            }
            if (step == 6) {

                if (!(input.getDistance() >= 50)) {
                    input.move(50);
                }
                else {
                    input.move(0);
                    step++;
                }


            }

            BotTelemetry.addData("Step:", step);
            BotTelemetry.addData("Encoder Distance (in inches)", input.getDistance());
            BotTelemetry.update();
        }


    }
}
