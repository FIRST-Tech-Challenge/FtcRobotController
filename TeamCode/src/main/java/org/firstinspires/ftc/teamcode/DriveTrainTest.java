package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
@Disabled
public class DriveTrainTest extends LinearOpMode {


    private final DriveTrain dt = new DriveTrain();

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        boolean toggle = false;
        while (opModeIsActive()) {
            if(toggle){
                dt.FieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            }else{
                dt.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
            }
            if(gamepad1.start){
                toggle = !toggle;
            }
            while(gamepad1.a){
                dt.getToAngle(new double[]{.000001, 0.00004, 0.00001}, 90);
            }
        }
    }
}

