package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.tseDepositor;

@Autonomous(name="Tse Depositor Test")
@Disabled

public class TseDepositorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        tseDepositor tse = new tseDepositor(false);
        /*tse.moveTseDepositerTape("Time Based",69,  1);
        sleep(10000);
        tse.moveTseDepositerTape("Time Based",69,  0);*/
        tse.setTseCrServoPower(0.5);
        sleep(9000);
        tse.setTseCrServoPower(0.0);
        tse.retract();
        sleep(5000);
    }
}
