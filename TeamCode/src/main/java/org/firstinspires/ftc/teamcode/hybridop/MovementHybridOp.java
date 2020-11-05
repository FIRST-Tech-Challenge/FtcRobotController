package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.HybridOpController;

@TeleOp(name="Movement HyOp")
public class MovementHybridOp extends MovementHardware implements HybridOp {

    final double POWER_SCALE_FACTOR = 0.5;
    final double ROTATE_SCALE_FACTOR = 0.4;
    boolean a_press = false;
    private ActionSequence movementActionSequence = new MovementActionSequence();

    @Override
    public void init() {
        super.init();
        this.hybridOpController = new HybridOpController(this, this);
    }

    @Override
    public void loop() {
        this.localize();
        this.hybridOpController.loop();
    }

    @Override
    public void hybrid_loop() {
        this.localizer.telemetry(telemetry);
        if (gamepad1.a) {
            if (!a_press) {
                if (this.hybridOpController.isAutonomous()) {
                    this.hybridOpController.stopAutonomous();
                } else {
                    this.hybridOpController.executeActionSequence(movementActionSequence, true);
                }
                a_press = true;
            }
        } else {
            a_press = false;
        }

    }

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
        double power = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double angle = Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        double rotate = gamepad1.left_stick_x;

        power *= POWER_SCALE_FACTOR;
        rotate *= ROTATE_SCALE_FACTOR;
        telemetry.addData("power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("rotate", rotate);
        omniDrive.move(power, angle, rotate);
    }
}
