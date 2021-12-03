package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Driving;
import org.firstinspires.ftc.teamcode.systems.System;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="FTC ROBOT CONTROLLER")
public class Controller extends LinearOpMode {

    public void runOpMode(){

        List<System> systems = new ArrayList<>();

        // add the systems
        systems.add(new Driving(hardwareMap, this));
        // systems.add(new DuckSpinner(hardwareMap, this));
        // systems.add(new Arm(hardwareMap, this));

        for (System s : systems)
            s.init();

        telemetry.addData("Starting!", "Press start to START ;-;");

        telemetry.update();

        waitForStart();
        while(opModeIsActive()){

            // dropped time counting -> no need

            for (System s: systems)
                s.update();
            telemetry.update();

        }


    }

}
