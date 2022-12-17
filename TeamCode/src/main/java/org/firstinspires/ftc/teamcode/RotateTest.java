package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RotateTest",group="a")
public class RotateTest extends DriveMethods{
    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();
        double rotateDeg = 0;
        waitForStart();
        while(opModeIsActive()){
//            if(gamepad1.dpad_up){
//                rotateDeg += 1;
//            } if(gamepad1.dpad_down) {
//                rotateDeg -= 1;
//            }
            rotateDeg += gamepad1.left_stick_x;
            if(gamepad1.a) {
                driveForDistance(0, Variables.Direction.ROTATE,0,rotateDeg);
            }
            telemetry.addLine("Rotate Degree: " + rotateDeg);
            telemetry.update();
        }
    }
}
