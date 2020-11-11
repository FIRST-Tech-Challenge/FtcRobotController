package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.HybridOpController;

@TeleOp(name="Ultimate Goal Tele Op")
public class UltimateGoalHybridOp extends UltimateGoalHardware implements HybridOp {

    boolean spinners = false;
    boolean bPress = false;
    boolean upPress = false;
    boolean downPress= false;
    double spinnerPower = 1;
    double spinner_increment = 0.05;

    @Override
    public void hybrid_loop() {

    }

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
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
            shooterLeft.setPower(-spinnerPower);
            shooterRight.setPower(spinnerPower);
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        if (gamepad1.a) {
            escalator.setPower(1);
        } else {
            escalator.setPower(0);
        }

        if (gamepad1.y) {
            collector.setPower(1);
        } else {
            collector.setPower(0);
        }
    }

    @Override
    public void init() {
        super.init();
        this.hybridOpController = new HybridOpController(this, this);
    }

    @Override
    public void loop() {
        this.hybridOpController.loop();
    }
}
