package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;


@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class DriveTrainTest extends LinearOpMode {


    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
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
                dt.getToAngle(new double[]{1, 0, 2}, 90);
            }
            if(gamepad1.options){
                dt.reInitFieldCentric();
            }
        }
    }
}

