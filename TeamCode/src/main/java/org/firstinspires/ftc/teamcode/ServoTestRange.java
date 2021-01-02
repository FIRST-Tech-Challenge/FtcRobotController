package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test Range", group = "Linear Opmode")
//@Disabled
public class ServoTestRange extends LinearOpMode {

    public Servo servo = null;

    private double _maxOutPos = 0.05;
    private double _maxInPos = 1.0;
    private double _servoPosition = _maxInPos;
    private boolean _servoIn = true;
    private boolean _dpadUp = false;
    private boolean _dpadDown = false;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "SlideServo");
        servo.setPosition(_servoPosition);

        // Wait for the start button
        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.dpad_up
                    //&& !_dpadUp
                    ) {
                _servoPosition =  _servoPosition - 0.05;
                sleep(250);
                //_dpadUp = gamepad1.dpad_up;
            } else if (gamepad1.dpad_down
                    //&& !_dpadDown
                    ) {
                _servoPosition = _servoPosition + 0.05;
                sleep(250);
                //_dpadDown = gamepad1.dpad_down;
            }

            servo.setPosition(_servoPosition);

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
            idle();

        }
    }

}