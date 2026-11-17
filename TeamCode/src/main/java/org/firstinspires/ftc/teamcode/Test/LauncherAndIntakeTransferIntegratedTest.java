package org.firstinspires.ftc.teamcode.Test;

import static org.firstinspires.ftc.teamcode.Launcher.Flywheel.targetRPM;
import static org.firstinspires.ftc.teamcode.Test.IntakeTest.targetVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;
import org.firstinspires.ftc.teamcode.Intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.Launcher.Flywheel;
import org.firstinspires.ftc.teamcode.Launcher.LauncherServo;
import org.firstinspires.ftc.teamcode.Launcher.Turret;
import org.firstinspires.ftc.teamcode.Transfer.TransferFSM;
import org.firstinspires.ftc.teamcode.Transfer.TransferMotorFSM;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(group = "Launcher", name = "Full Launcher & Intake Transfer")
public class LauncherAndIntakeTransferIntegratedTest extends LinearOpMode {

    public NewIntakeTransferHWmap hwMap;
    public Turret turret;
    public Flywheel flywheel;
    public LauncherServo launcherServo;
    public IntakeFSM intakeFSM;
    public TransferFSM transferFSM;
    public TransferMotorFSM transferMotorFSM;
    private GamepadEx gamepad;
    Logger logger;

    private BufferedWriter csvWriter;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hwMap = new NewIntakeTransferHWmap(hardwareMap);
        // launcherServo = new LauncherServo(telemetry, hwMap);
        flywheel = new Flywheel(hwMap, telemetry);
        // turret = new Turret(hwMap, telemetry);
        gamepad = new GamepadEx(gamepad1);
        logger = new Logger(telemetry);
        transferFSM = new TransferFSM(hwMap, telemetry);
        intakeFSM = new IntakeFSM(hwMap, telemetry, transferFSM);
        transferMotorFSM = new TransferMotorFSM(hwMap, telemetry);


        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        String fileName = "/sdcard/FIRST/motor_velocity_" + timestamp + ".csv";

        try {
            csvWriter = new BufferedWriter(new FileWriter(fileName));
            csvWriter.write("Time(ms),FlywheelCurrentRPM,FlywheelTargetRPM,Current(amps)\n");
        } catch (IOException e) {
            telemetry.addData("CSV Error", e.getMessage());
        }

        telemetry.update();

        waitForStart();

        long startTime = System.currentTimeMillis();

        try {
            while (opModeIsActive()) {
                long currentTime = System.currentTimeMillis() - startTime;

                telemetry.addData("Current RPM", flywheel.getFlywheelMotor().getRPM());
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Current (amps)", flywheel.getFlywheelMotor().getCurrent());


                // launcherServo.updatePID();
                flywheel.update();
                // turret.updatePID();
                gamepad.readButtons();
                intakeFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.Y), gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT));
                transferFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT));

                if (csvWriter != null) {
                    try {
                        csvWriter.write(currentTime + "," +
                                flywheel.getFlywheelMotor().getRPM() + "," +
                                targetRPM + "," +
                                flywheel.getFlywheelMotor().getCurrent() + "\n");
                    } catch (IOException e) {
                        telemetry.addData("CSV Write Error", e.getMessage());
                    }
                }

                telemetry.update();
            }
        } finally {
            if (csvWriter != null) {
                try {
                    csvWriter.flush();
                    csvWriter.close();
                    telemetry.update();
                } catch (IOException ignored) {
                }
            }
        }
    }
}
