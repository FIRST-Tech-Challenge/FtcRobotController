package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.apache.commons.math3.analysis.function.Abs;

@TeleOp(name="Encoder Offset Tester")

public class EncoderOffsetTesting extends LinearOpMode {



    public AbsoluteAnalogEncoder efrontRight;
    public AbsoluteAnalogEncoder efrontLeft;
    public AbsoluteAnalogEncoder ebackLeft;
    public AbsoluteAnalogEncoder ebackRight;

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        efrontRight =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontRight"), 3.3);
        efrontLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontLeft"), 3.3);
        ebackLeft =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackLeft"), 3.3);
        ebackRight=  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackRight"), 3.3);

        waitForStart();

        while (opModeIsActive()) {
            double frontRightV = efrontRight.getCurrentPosition();
            double frontLeftV = efrontLeft.getCurrentPosition();
            double backLeftV = ebackLeft.getCurrentPosition();
            double backRightV = ebackRight.getCurrentPosition();


            telemetry.addData("You can", "start aligning the left and right wheels with a straightedge!");

            telemetry.addData("frontRight", frontRightV + "rad");
            telemetry.addData("frontLeft", frontLeftV + "rad");
            telemetry.addData("backLeft", backLeftV + "rad");
            telemetry.addData("backRight", backRightV + "rad");
            telemetry.update();
        }
    }
}
