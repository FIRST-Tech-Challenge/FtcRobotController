package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo And Slider")
public class ServoAndSlider extends LinearOpMode {
    static final double INCREMENT = 0.01;
    static final int CYCLE_MS = 50;
    static final double MAX_POS_SERVO = 0.9;
    static final double MIN_POS_SERVO = 0.6;
    static final double MAX_POS_SLIDER = 1.20;
    static final double MIN_POS_SLIDER = 0.1;

    Servo servo;
    DcMotor dcMotor;

    double posServo = MIN_POS_SERVO;
    double posSlider = 0;
    boolean sliderUp = true;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "dropperServo");
        dcMotor = hardwareMap.get(DcMotor.class, "sliderMotor");

        telemetry.addData(">", "Press Start to scan Motor and Servo.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update slider position
            if (sliderUp) {
                posSlider += INCREMENT;
                if (posSlider >= MAX_POS_SLIDER) {
                    sliderUp = false;  // Slider reached the top
                    posServo = MAX_POS_SERVO;  // Drop the item by moving servo to max position
                }
            } else {
                posSlider -= INCREMENT;
                if (posSlider <= MIN_POS_SLIDER) {
                    sliderUp = true;  // Slider reached the bottom, start moving up again
                    posServo = MIN_POS_SERVO;  // Retract the servo to its minimum position
                }
            }

            // Set motor power based on slider direction
            dcMotor.setPower(sliderUp ? 1.0 : -1.0);

            // Set servo position
            servo.setPosition(posServo);

            telemetry.addData("Servo Position", "%5.2f", posServo);
            telemetry.addData("Slider Position", "%5.2f", posSlider);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            sleep(CYCLE_MS);
        }

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}