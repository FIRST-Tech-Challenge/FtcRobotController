package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Transfer {

    private DcMotor transferMotor = null;

    final private double transferSpeed = 1.0;

    // initialization of transferMotor
    public Transfer(LinearOpMode opMode){
        transferMotor = opMode.hardwareMap.dcMotor.get("TransferMotor");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startTransfer() {
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setPower(transferSpeed);
    }

    public void stopTransfer() {
        transferMotor.setPower(0);
    }
}
