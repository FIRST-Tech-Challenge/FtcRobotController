package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.tseDepositor;

@Autonomous(name="Tse Depositor Test")

public class TseDepositorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        tseDepositor tse = new tseDepositor(this);
        tse.moveTseDepositerTape("Time Based",69,  1);
        sleep(10000);
        tse.moveTseDepositerTape("Time Based",69,  0);
    }
}
