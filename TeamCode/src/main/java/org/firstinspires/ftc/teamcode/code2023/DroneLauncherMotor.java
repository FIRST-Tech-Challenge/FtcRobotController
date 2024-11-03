package org.firstinspires.ftc.teamcode.code2023;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class DroneLauncherMotor {
    private final DcMotor droneLauncherMotor;

    private DroneLauncherMotor(DcMotor droneLauncherMotor) {
        this.droneLauncherMotor = droneLauncherMotor;
        this.droneLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.droneLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public DroneLauncherMotor(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().dcMotor.get("planeLauncher"));
    }

    public void setPower(double power) {
        droneLauncherMotor.setPower(power);
    }
    public void off() {
        droneLauncherMotor.setPower(0);
    }

    public int getPosTicks() {
        return droneLauncherMotor.getCurrentPosition();
    }

}
