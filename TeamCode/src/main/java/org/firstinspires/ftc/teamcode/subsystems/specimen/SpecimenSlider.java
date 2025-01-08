package org.firstinspires.ftc.teamcode.subsystems.specimen;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;


public class SpecimenSlider extends SonicSubsystemBase {

    private CRServo servo1;

    private CRServo servo2;

    private Telemetry telemetry;

    public SpecimenSlider(HardwareMap hardwareMap, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.servo1  = hardwareMap.get(CRServo.class,"SpecimenServo1");
        this.servo2  = hardwareMap.get(CRServo.class,"SpecimenServo2");

        this.telemetry = telemetry;
    }

    boolean isExpanding = false;

    public void ToggleExpand() {
        if(isExpanding) {
            Hold();
        } else {
            Expand();
        }

        isExpanding = !isExpanding;
    }

    boolean isCollapsing = false;

    public void ToggleCollapse() {
        if(isCollapsing) {
            Hold();
        } else {
            Collapse();
        }

        isCollapsing = !isCollapsing;
    }

    public void Expand() {
        this.servo1.setPower(-1);
        this.servo2.setPower(1);
    }

    public void Collapse() {
        this.servo1.setPower(1);
        this.servo2.setPower(-1);
    }

    public void Hold() {
        this.servo1.setPower(0.1);
        this.servo2.setPower(0.1);
    }

    public void AutoExpand() {
        this.servo1.setPower(-1);
        this.servo2.setPower(1);

        this.servo1.setPower(0);
        this.servo2.setPower(0);
    }
}