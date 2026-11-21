package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.IntakeTransferHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;
import org.firstinspires.ftc.teamcode.Intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.Transfer.TransferFSM;

@Config
@TeleOp(group = "Intake Transfer", name = "Full Intake Transfer")
public class IntakeTransferTest extends LinearOpMode {


    private NewIntakeTransferHWmap hwmap;
    private GamepadEx gamepad;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
/*    private Logger logger;*/

    @Override

    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
 /*       this.logger = logger;*/
        hwmap = new NewIntakeTransferHWmap(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
  /*      logger = new Logger(telemetry);*/
        transferFSM = new TransferFSM(hwmap, telemetry);
        intakeFSM = new IntakeFSM(hwmap, telemetry, transferFSM);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
  /*          logger.print();*/
          /*  intakeFSM.log();*/
       /*     transferFSM.log();*/
         /*   logger.updateLoggingLevel(gamepad1.dpad_right);*/
            gamepad.readButtons();
            intakeFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.Y), (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)));
            transferFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT));

        }

    }
}