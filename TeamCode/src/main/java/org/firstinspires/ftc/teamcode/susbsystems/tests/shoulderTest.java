package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Arm725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shoulder Test", group = "SubsysTest")
public class shoulderTest extends LinearOpMode{
    Arm725 shoulder;
    ButtonBlock runToPos, stopArm, dpadUp, dpadDown,runToHeight, runToHome;
    private Servo wrist;

    private static final double MANUAL_DEADBAND = 0.05;
    private static final double DEG_TO_TICKS = 8.05;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        shoulder = new Arm725(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "wrist");

        runToPos = new ButtonBlock().onTrue(() -> {shoulder.GoToAngle(targetPos);});
        stopArm = new ButtonBlock().onTrue(() -> {shoulder.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 3;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 3;});
        runToHeight = new ButtonBlock().onTrue(() -> {shoulder.GoToHeight(targetPos);});
        runToHome = new ButtonBlock().onTrue(() -> {shoulder.Home();});


        telemetry.addLine("Initialized");
        telemetry.update();

        while (!shoulder.Homed() && !opModeIsActive() && !isStopRequested()) {
            runToHome.update(gamepad1.x);
            handleManualShoulderControl();
            shoulder.Update();
        }
        waitForStart();
        wrist.setPosition(1);
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            runToHeight.update(gamepad1.y);
            stopArm.update(gamepad1.b);
            runToHome.update(gamepad1.x);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            handleManualShoulderControl();
            shoulder.Update();

            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target Angle");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("X: Home");
            telemetry.addLine("Y: Go to Target Height");
            telemetry.addLine("Right Stick Y: Up/Down");
            telemetry.addLine();

            // Use this section to verify that an angle command converts to the
            // expected encoder target and that the arm reaches that target.
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  shoulder.GetPower());
            telemetry.addData("Homed", shoulder.Homed());
            telemetry.addData("Is Busy", shoulder.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos","Deg: "+shoulder.GetPos()+" Raw: "+shoulder.GetRawPos()+" Height: "+shoulder.GetHeight());
            telemetry.addData("Active Target Pos", "Deg: "+shoulder.GetTargetPos()+" Raw: "+shoulder.GetRawTargetPos()+" T_Height: "+shoulder.GetTargetHeight());

            // Error should move toward zero during RUN_TO_POSITION. If it stays
            // large while power is applied, the motor/controller is not reaching
            // the commanded target even though the command was correct.
            telemetry.addData("Target Error", "Deg: %.2f Raw: %.0f",
                    shoulder.GetTargetErrorDegrees(),
                    shoulder.GetTargetErrorTicks());

            // This is the expected raw target from the displayed Target Pos.
            // It should match Active Target Raw after pressing A.
            telemetry.addData("Expected Raw From Target", "%.0f", targetPos * DEG_TO_TICKS);

            // Quick pass/fail check for angle commands. For a normal GoToAngle
            // verification, wait until Busy is false and this says YES.
            telemetry.addData("Angle Target Reached",
                    Math.abs(shoulder.GetTargetErrorTicks()) <= 5 ? "YES" : "NO");
            telemetry.update();
        }
    }

    private void handleManualShoulderControl() {
        double shoulderStick = gamepad1.right_stick_y;
        if (Math.abs(shoulderStick) < MANUAL_DEADBAND) {
            shoulderStick = 0;
        }

        // Only interrupt RUN_TO_POSITION when the driver intentionally moves the stick.
        if (!shoulder.IsBusy()) {
            shoulder.SetPower(-shoulderStick);
        } else if (Math.abs(shoulderStick) > 0) {
            shoulder.Stop();
            shoulder.SetPower(-shoulderStick);
        }
    }
}
