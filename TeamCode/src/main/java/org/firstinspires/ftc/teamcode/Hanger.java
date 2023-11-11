package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanger {
    private final DcMotor hangerMotor;
    private final int countsPerRev = 1531; //not sure of this value
    private final double HANGER_REVOLUTIONS = 0.2; //need to test to see how many revolutions


    public Hanger(HardwareMap hardwareMap) {
        hangerMotor = hardwareMap.dcMotor.get("hangerMotor");
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHangerPower(Button button) {
        if(button.Pressed()){ //button is pressed
            int targetPosition = (int)(countsPerRev * HANGER_REVOLUTIONS); //cast to int
            if (hangerMotor.getCurrentPosition() < targetPosition) {
                // Forward direction
                hangerMotor.setTargetPosition(targetPosition);
            } else {
                // Reverse direction
                hangerMotor.setTargetPosition(0);
            }
            hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangerMotor.setPower(0.3);
        } else {
            hangerMotor.setPower(0);
            hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
