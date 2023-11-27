package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tools.TelemetryManager;

public class Hanger {
    public final TouchSensor hangerTouchUp;
    private final DcMotor hangerMotor;
    private final int countsPerRev = 1531; //not sure of this value
    private final double HANGER_REVOLUTIONS = 0.4; //need to test to see how many revolutions
    private final Button handlerDPadDown, handlerDPadUp;


    public Hanger(HardwareMap hardwareMap, Button handlerDPadDown, Button handlerDPadUp) {
        hangerMotor = hardwareMap.dcMotor.get("skyHookMotor");
        hangerTouchUp = hardwareMap.touchSensor.get("skyHookTouchUp");
        hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.handlerDPadDown = handlerDPadDown;
        this.handlerDPadUp = handlerDPadUp;
    }

    public void update(Button button) {
        TelemetryManager.getTelemetry().addData("Hanger Pos: ", hangerMotor.getCurrentPosition());
        if(handlerDPadUp.On()){
            if(hangerTouchUp.isPressed()) {
                hangerMotor.setPower(0);
            }
            else {
                int targetPosition = (int) (countsPerRev * HANGER_REVOLUTIONS); // cast to int
                hangerMotor.setTargetPosition(-targetPosition);
                hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangerMotor.setPower(0.2);
            }
        }
        else if(handlerDPadDown.On()){ // button is pressed
            //int targetPosition = (int)(countsPerRev * HANGER_REVOLUTIONS); // cast to int
            hangerMotor.setTargetPosition(0);
            hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangerMotor.setPower(0.2);
        }
        else {
            hangerMotor.setPower(0);
            //hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
