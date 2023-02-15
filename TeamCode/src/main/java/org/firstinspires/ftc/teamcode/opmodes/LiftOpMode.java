package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames.LIFT_ENCODER;
import static org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames.LIFT_MOTOR;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import ftc.rogue.blacksmith.util.SignalEdgeDetector;

@Config
@TeleOp
public class LiftOpMode extends LinearOpMode {
    public static double P = 0.001;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        SignalEdgeDetector dpadUp = new SignalEdgeDetector(() -> gamepad1.dpad_up);
        SignalEdgeDetector dpadDown = new SignalEdgeDetector(() -> gamepad1.dpad_down);

        ToggleButtonReader aToggle = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.A);

        DcMotorSimple liftMotor = hardwareMap.get(DcMotorSimple.class, LIFT_MOTOR);
        Motor liftEncoder = new Motor(hardwareMap, LIFT_ENCODER);

        double targetHeight = 0;

        PIDFController controller = new PIDFController(P, I, D, F);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double correction;
            dpadUp.update();
            dpadDown.update();
            if (dpadUp.risingEdge()) {
                targetHeight = 700;
            }
            if (dpadDown.risingEdge()) {
                targetHeight = 0;
            }
            if(aToggle.getState()){
                correction = controller.calculate(-liftEncoder.getCurrentPosition(), targetHeight);
            }
            else{
                correction = gamepadEx.getLeftY()/2;
            }





            telemetry.addData("Correction", correction);
            telemetry.update();

            liftMotor.setPower(correction);
        }
    }
}
