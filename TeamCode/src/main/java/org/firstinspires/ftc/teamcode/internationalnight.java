
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class internationalnight extends LinearOpMode {
    public DcMotor spin;
    public void runOpMode(){
        spin= hardwareMap.get(DcMotor.class,"Spin");
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()){
            sleep(1000);
            spin.setTargetPosition(591);
            spin.setPower(.2);
            while (spin.isBusy()&&opModeIsActive()){

            }
            sleep(1000);
            spin.setTargetPosition(0);
            spin.setPower(.2);
            while (spin.isBusy()&&opModeIsActive()){

            }

        }
    }

}
