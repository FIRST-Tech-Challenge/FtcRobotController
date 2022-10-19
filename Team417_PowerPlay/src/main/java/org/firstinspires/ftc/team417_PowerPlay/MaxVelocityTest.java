package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MaxVelocityTest extends BaseOpMode {
    double currentVelocity;
    double maxVelocity;
    double runtime = 1.0;
    DcMotorEx frontLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        //frontLeft = hardwareMap.dcMotor.get
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        motorFL.setPower(1);
        motorFR.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
        while (opModeIsActive() && timer.seconds() < runtime) {
            currentVelocity = motorFL.getPower();


        }
    }
}
