package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    Telemetry telemetry;
    String ROBOT_CAPTION = "Robot Status";
    DriveTrain driveTrain;
    LinearOpMode opMode;
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();
    HardwareInnov8Robot hwmap;

    public Robot(Telemetry telemetry, HardwareMap hwmap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.hwmap = new HardwareInnov8Robot(hwmap);
        this.telemetry = telemetry;
        driveTrain = new DriveTrain(this.telemetry, this.hwmap, this.opMode);
        this.telemetry.addData(ROBOT_CAPTION, "ready to go");
        this.telemetry.update();
    }

    public double getConfi() {
        this.telemetry.addData(ROBOT_CAPTION, "getting confi");
        double confi = 0.98;
        return confi;
    }

    public void stop() {
        this.telemetry.addData(ROBOT_CAPTION, "free");

        this.telemetry.update();
    }

    public void teleop(Gamepad gamepad1, Gamepad gamepad2) {

        while (this.opMode.opModeIsActive()) {
            this.telemetry.addData(ROBOT_CAPTION, "teleop-ing");
            driveTrain.teleopUpdate(gamepad1, gamepad2);
            this.telemetry.update();
        }
    }
    

    public void forwardTwelveInches() {
        
        driveTrain.goForward(12);
    }

    public void straightRight(){
        driveTrain.goForward(15);
        driveTrain.turn(-90);
        driveTrain.goForward((30));
    }
}
