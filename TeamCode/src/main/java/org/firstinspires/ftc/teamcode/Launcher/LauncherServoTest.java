package org.firstinspires.ftc.teamcode.Launcher;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.NewAxonServo;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;

@Config
public class LauncherServo {

    HWMap hwMap;
    private NewAxonServo pitchServo;
    public static double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCE = 1;
    public static double P = 0.1, I = 0, D = 0, F = 0;
    public static double gearRatio = 1.0 / 12.0;

    public static double UPPER_HARD_STOP = 29;
    public static double LOWER_HARD_STOP = 10;
    Telemetry telemetry;

    public LauncherServo(Telemetry telemetry,  HWMap hwMap) {
        pitchServo = new NewAxonServo(hwMap.getPitchServo(), hwMap.getPitchEncoder(), false, false, 0, gearRatio);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidfController = new PIDFController(P, I, D, F);
        pidfController.setTolerance(TOLERANCE);
    }

    public void updatePID() {
        if (targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        } else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }
        pidfController.setPIDF(P, I, D, F);
        pidfController.setTolerance(TOLERANCE);
        pitchServo.readPos();

        double error = targetAngle - pitchServo.getScaledPos();
        double power = pidfController.calculate(pitchServo.getScaledPos(), targetAngle);

        pitchServo.set(power);

        telemetry.addData("error", error);
        telemetry.addData("power", power);
    }
}
