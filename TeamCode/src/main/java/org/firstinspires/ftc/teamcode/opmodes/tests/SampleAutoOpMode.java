package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Config
@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private TelemetryPacket packet;
    private FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);


        dash = FtcDashboard.getInstance();
        //telemetry = dash.getTelemetry();
        packet = new TelemetryPacket();

        telemetry.addData("Hello","Hello");

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


mecanumCommand.setFinalPosition(0,0,5,0);
            if(mecanumCommand.positionNotReachedYet()) {
                mecanumCommand.processPIDUsingPinpoint();
                telemetry.addData("Hello", "Hello");

            }
            else {
                stopRobot();
            }

            // run processes
            updateTelemetry();
            mecanumCommand.motorProcess();


        }

    }


    public void updateTelemetry() {
        packet.put("x: ", mecanumCommand.getOdoX());
        packet.put("y: ", mecanumCommand.getOdoY());
        packet.put("theta: ", mecanumCommand.getOdoHeading());
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Theta: ", mecanumCommand.getOdoHeading());

        telemetry.update();
    }



    private void stopRobot() {
        mecanumCommand.stop();
    }


    public void processPinPoint() {
        mecanumCommand.deadReckoning();

    }

    }


