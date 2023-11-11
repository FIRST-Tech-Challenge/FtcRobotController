package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    private final DcMotor intakeMotor;
    private double INTAKE_POWER;
    private boolean runIntake = false;

    public Intake() {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void setIntakePower(Button button) {
        if (button.Pressed()) {
            runIntake = !runIntake;
            if (runIntake){
                intakeMotor.setPower(INTAKE_POWER);
            }
            else{
                intakeMotor.setPower(0);
            }
        }
    }
}
