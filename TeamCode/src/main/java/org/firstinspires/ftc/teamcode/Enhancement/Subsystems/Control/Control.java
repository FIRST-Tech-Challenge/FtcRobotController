package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Control;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

public class Control extends Subsystem {
    DcMotorEx intake;

    public Control(QuickTelemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer, DcMotorEx intake) {
        super(telemetry, hardwareMap, timer);
        this.intake = intake;
        telemetry.telemetry(2, "Control initialized", "Control initialized");
    }

    public void setIntakeDirection(boolean status) {      // simplified so only one method is needed for intake.
        boolean isIntakeOn = intake.isMotorEnabled();     // parameter true is for regular motion, false is for reverse.
        if (status) {                              // toggles on/off when invoked while in opposite state, e.g.
            if (!isIntakeOn) {                            // turns off when previously on, turns on when previously off.
                intake.setPower(1.0);
                isIntakeOn = true;
            } else {
                intake.setPower(0.0);
                isIntakeOn = false;
            }
        } else if (!status) {
            if (!isIntakeOn) {
                intake.setPower(-0.8);
                isIntakeOn = true;
            } else {
                intake.setPower(0.0);
                isIntakeOn = false;
            }
        }
    }
}
