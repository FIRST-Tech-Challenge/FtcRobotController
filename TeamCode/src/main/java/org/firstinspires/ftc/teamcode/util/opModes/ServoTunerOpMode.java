package org.firstinspires.ftc.teamcode.util.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.SympleServo;

import java.util.HashMap;
import java.util.Map;

@Config
@TeleOp(name = "Servo Tuner", group = "tune")
public class ServoTunerOpMode extends CommandOpMode {
    private static final HashMap<Integer, ServoMap> SERVOS = new HashMap<>();

    static {
        ServoMap[] motors = ServoMap.values();
        for (int i = 0; i <motors.length; i++) {
            SERVOS.put(i, motors[i]);
        }
    }
    public static int servoId = 0;
    public static double angle = 0;

    private GamepadEx gamepadEx;
    private ServoMap servoName;
    private SympleServo servo;

    @Override
    public void initialize() {
        this.gamepadEx = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initializeLoop() {
        telemetry.addData("Servos List:", " ");
        for(Map.Entry<Integer, ServoMap> entry : SERVOS.entrySet()) {
            telemetry.addData(entry.getKey().toString(), entry.getValue().getId());
        }

        ServoMap currentServo = SERVOS.get(servoId);
        if(currentServo != null) {
            this.servoName = currentServo;
        }

        telemetry.addData("Current Servo", servoName != null ? servoName.getId() : "Unknown");
        telemetry.update();
    }

    private void postInitialize() {
        servo = new SympleServo(hardwareMap, servoName.getId(), 0, 300, AngleUnit.DEGREES);

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> servo.turnToAngle(angle)));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("angle", angle);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        this.initialize();

        // runs when in init mode
        while (this.opModeInInit() && !this.isStopRequested()) {
            initializeLoop();
        }

        this.waitForStart();

        postInitialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            this.run();
        }

        this.reset();
    }
}
