package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotBasic extends Component {
    private LiftBasic lift = null;
    private Servo grabber = null;
    private double grabberOpenPos = 0.0;
    private double grabberClosedPos = 1.0;

    public BotBasic(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        lift = new LiftBasic(hardwareMap, telemetry);
    }

    public void liftManualUp(double power) {
        lift.manualUp(power);
    }

    public void liftManualDown(double power) {
        lift.manualDown(power);
    }

    public void liftStop() {
        lift.stop();
    }

    public void log() {
        lift.log();
    }
}
