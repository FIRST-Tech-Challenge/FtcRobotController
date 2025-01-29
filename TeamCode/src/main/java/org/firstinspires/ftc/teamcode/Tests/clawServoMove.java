package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Servos;

@Config
@Autonomous(name = "servo test")
public class clawServoMove extends LinearOpMode {

    Servos servos;

    public static int position;

    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap);

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        waitForStart();

//        servos.setServoPosition(Servos.Type.Wrist,0);
//        sleep(3000);

        while(opModeIsActive())
        {


            servos.setServoPosition(Servos.Type.Wrist, position);
//            sleep(500);
//            servos.setServoPosition(Servos.Type.Wrist,180);
//            sleep(500);
//            servos.setServoPosition(Servos.Type.Wrist,270);
//            sleep(500);
//            servos.setServoPosition(Servos.Type.Wrist,0);
//            sleep(500);

            BotTelemetry.addData("Wanted Position", position);
            BotTelemetry.addData("Wrist Position", servos.getServoPosition(Servos.Type.Wrist));
            BotTelemetry.update();
        }
    }
}