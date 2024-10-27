package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

public class DroneLauncher {

    private final OpModeUtilities opModeUtilities;
    public DcMotor wheel;
    public Servo engageServo;

    private static final double LAUNCHER_SERVO_DISENGAGE_POS = 0.73;
    private static final double LAUNCHER_SERVO_ENGAGE_POS = 0.45;

    private static final double P_CONSTANT = 0.004;

    public DroneLauncher(OpModeUtilities opModeUtilities) {

        this.opModeUtilities = opModeUtilities;
        setUpHardware();

    }

    private void setUpHardware() {

        wheel = opModeUtilities.getHardwareMap().dcMotor.get("planeLauncher");
        engageServo = opModeUtilities.getHardwareMap().servo.get("planeLauncherServo");

        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}