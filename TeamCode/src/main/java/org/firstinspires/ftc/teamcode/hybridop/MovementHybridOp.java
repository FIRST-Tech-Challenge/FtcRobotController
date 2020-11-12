package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;

@TeleOp(name="Movement HyOp")
public class MovementHybridOp extends MovementHardware implements HybridOp {

    final double POWER_SCALE_FACTOR = 0.5;
    final double ROTATE_SCALE_FACTOR = 0.4;
    private ActionSequence movementActionSequence = new MovementActionSequence();

    @Override
    public void init() {
        super.init();
        this.initializeForHybridOp(this);
    }

    @Override
    public void run_loop() {
        this.localizer.telemetry(telemetry);
        if (this.gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a)) {
            this.hybridOpExecutor.executeActionSequence(movementActionSequence, true, true);
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
