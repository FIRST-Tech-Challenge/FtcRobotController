package org.firstinspires.ftc.masters.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Autonomous(name="specimen Blue")
public class SpecimenBlue extends LinearOpMode {

    Init init;
    DriveTrain driveTrain;
    Outake outake;
    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        init = new Init(hardwareMap);
        driveTrain = new DriveTrain(init, telemetry);
        outake = new Outake(init, telemetry);
        intake = new Intake(init, telemetry);



        waitForStart();


    }
}
