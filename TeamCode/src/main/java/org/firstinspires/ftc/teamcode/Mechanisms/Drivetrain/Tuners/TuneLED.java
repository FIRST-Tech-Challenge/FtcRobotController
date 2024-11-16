package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PWMOutput;

public class TuneLED {
    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="White RGB LED Control")
    public class WhiteRGBLEDControl extends LinearOpMode {

        private PWMOutput redLED, greenLED, blueLED;
        private int whiteIntensity = 255;

        @Override
        public void runOpMode() {
            redLED = hardwareMap.get(PWMOutput.class, "redLED");
            greenLED = hardwareMap.get(PWMOutput.class, "greenLED");
            blueLED = hardwareMap.get(PWMOutput.class, "blueLED");

            waitForStart();


            setWhite(whiteIntensity);


            while (opModeIsActive()) {
                telemetry.addData("LED Status", "White Light ON");
                telemetry.update();
            }
        }

        private void setWhite(int intensity) {
            redLED.setPulseWidthOutputTime(intensity);
            greenLED.setPulseWidthOutputTime(intensity);
            blueLED.setPulseWidthOutputTime(intensity);
        }
    }

}
