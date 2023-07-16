package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class RetractOdo extends HWMap {

    public RetractOdo(Telemetry telemetry, HardwareMap hardwareMap){
        super(telemetry, hardwareMap);
    }

    // Servos on encoder wheels for retracting and un-retracting them
    public double LEFT_RETRACT_POS = 0.7;
    public double LEFT_UNRETRACT_POS = 0.25;
    public double RIGHT_RETRACT_POS = 0.0;
    public double RIGHT_UNRETRACT_POS = 0.6;
    public double FRONT_RETRACT_POS = 0.5;
    public double FRONT_UNRETRACT_POS = 1.0;

    public void retractOdometryServos() {
        leftServo.setPosition(LEFT_RETRACT_POS);
        rightServo.setPosition(RIGHT_RETRACT_POS);
        //frontServo.setPosition(FRONT_RETRACT_POS);
    }

    public void unretractOdometryServos() {
        leftServo.setPosition(LEFT_UNRETRACT_POS);
        rightServo.setPosition(RIGHT_UNRETRACT_POS);
        //frontServo.setPosition(FRONT_UNRETRACT_POS);
    }

}
