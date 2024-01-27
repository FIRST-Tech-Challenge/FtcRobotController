package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tools.Button;

public class Intake {
    private final DcMotor intakeMotor;
    private boolean runIntake = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakePower(Button button) {
        if (button.Pressed()) {
            runIntake = !runIntake;
            if (runIntake){
                double INTAKE_POWER = 1;
                intakeMotor.setPower(INTAKE_POWER);
            }
            else{
                intakeMotor.setPower(0);
            }
        }
    }
}
