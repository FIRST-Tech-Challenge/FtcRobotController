package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class LiftStateMachine extends StateMachine {

    /*private State.Transition.org.firstinspires.ftc.teamcode.tools.Action setMotor(DcMotor motor, int position){
        return new State.Transition.org.firstinspires.ftc.teamcode.tools.Action(()->{motor.setTargetPosition(position);
            return true;
        });
    }*/
    /*private void setPosServo(Servo servo, int pos, Telemetry telemetry) {
        int tolerance = 1;
        servo.setPosition(pos);
        int counter = 0; // counts the number of times the correctional loop runs
        // is my error greater or less than error
        while (Math.abs(pos - servo.getPosition()) >= tolerance && counter <= 5) {
            telemetry.addLine(servo + " correction started");
            // divide by two on the correctional value so that it does not overshoot by too much
            if (pos - servo.getPosition() > 0) {
                servo.setPosition(pos + (pos - servo.getPosition()) / 2);
            }
            if (pos - servo.getPosition() < 0) {
                servo.setPosition(pos + (servo.getPosition() - pos) / 2);
            }
            counter++;
            telemetry.addLine(servo + " corrected " + counter + " times");
        }
        telemetry.addLine(servo + " correction finished");
        telemetry.update();
    }

    // Method for motor correction
    private void setPosMotor(DcMotor motor, int pos, Telemetry telemetry) {
        int tolerance = 1;
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5); // Set some default power

        int counter = 0; // Counts the number of times the correctional loop runs

        while (Math.abs(pos - motor.getCurrentPosition()) > tolerance && counter <= 5) {
            telemetry.addLine(motor + " correction started");
            telemetry.update();

            int currentPosition = motor.getCurrentPosition();
            int error = pos - currentPosition;
            int halfError = error / 2;

            motor.setTargetPosition(currentPosition + halfError);

            while (*//*opModeIsActive() &&**//* motor.isBusy()) {
                // Optionally include idle() if in LinearOpMode
                telemetry.addData("Encoder Target", pos);
                telemetry.addData("Current Position", currentPosition);
                telemetry.update();
            }
            motor.setPower(0);

            counter++;
            telemetry.addData(motor + " corrected ", counter + " times");
            telemetry.update();
        }

        motor.setPower(0); // Ensure the motor is stopped at the end
        telemetry.addLine(motor + " correction finished");
        telemetry.update();
    }*/
}
