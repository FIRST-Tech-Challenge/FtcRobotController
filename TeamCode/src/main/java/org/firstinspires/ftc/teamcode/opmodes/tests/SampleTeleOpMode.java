package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private ElapsedTime timer;

    private ElapsedTime resetTimer;

    //OpModes should create the hardware object
    private Hardware hw;

    enum ROBOT_STATE{
        IDLE, SLOW
    }

    public ROBOT_STATE liftState = ROBOT_STATE.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(this, hw);
        while (opModeInInit()){
            telemetry.update();
        }

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            handleMovement();
            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }
        }
    }

    //
    private void handleMovement(){
        if (liftState == ROBOT_STATE.SLOW ){
            mecanumCommand.fieldOrientedMove(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.35);
        } else {
            mecanumCommand.fieldOrientedMove(-gamepad1.left_stick_y , gamepad1.left_stick_x , gamepad1.right_stick_x);
        }
    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");

        telemetry.update();
    }
}