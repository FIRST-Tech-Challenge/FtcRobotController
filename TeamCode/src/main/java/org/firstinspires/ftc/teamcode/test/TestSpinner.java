package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SpinnerHardware;

@TeleOp(name="Spinner Test")
public class TestSpinner extends SpinnerHardware {
//spinny test spinner lmao

    boolean spinners = false;
    boolean bPress = false;
    boolean upPress = false;
    boolean downPress= false;
    double spinnerPower = 1;
    double spinner_increment = 0.05;

    @Override
    public void loop() {
        telemetry.addData("spinner power", String.format("%.2f", spinnerPower));

        if (gamepad1.dpad_up) {
            if (!upPress) {
                upPress = true;
                double newPower = spinnerPower + spinner_increment;
                if (newPower <= 1) {
                    spinnerPower = newPower;
                }
            }
        } else {
            upPress = false;
        }

        if (gamepad1.dpad_down) {
            if (!downPress) {
                downPress = true;
                double newPower = spinnerPower - spinner_increment;
                if (newPower >= 0) {
                    spinnerPower = newPower;
                }
            }
        } else {
            downPress = false;
        }


        if (gamepad1.b) {
            if (!bPress) {
                bPress = true;
                spinners = !spinners;
            }
        } else {
            bPress = false;
        }

        if (spinners) {
            spinnerLeft.setPower(-spinnerPower);
            spinnerRight.setPower(spinnerPower);
        } else {
            spinnerLeft.setPower(0);
            spinnerRight.setPower(0);
        }

        if (gamepad1.a) {
            chainLift.setPower(1);
        } else {
            chainLift.setPower(0);
        }
    }
}
