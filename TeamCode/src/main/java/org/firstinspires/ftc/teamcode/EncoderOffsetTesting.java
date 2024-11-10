package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.apache.commons.math3.analysis.function.Abs;

@TeleOp(name="Encoder Offset Tester")

public class EncoderOffsetTesting extends LinearOpMode {

    public AbsoluteAnalogEncoder encoder =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "someHWMapName"), 3.3);

    @Override
    public void runOpMode() throws InterruptedException {
        //I HAVE NO IDEA HOW TO DO THIS
    }
}
