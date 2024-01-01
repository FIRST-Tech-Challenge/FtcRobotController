package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


//@Autonomous(name = "Class Test", group = "Tests")
@Disabled
public class ClassTest extends CSMethods {
    public void runOpMode() {
        setup(true);
        if (carWashMotor == null) {
            telemetry.addData("CarWashMotor", "null");

        }
        telemetry.update();
        sleep(5);

    }
}
