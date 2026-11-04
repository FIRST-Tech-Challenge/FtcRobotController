package org.nknsd.teamcode.programs.tests.allYears.physicalComponent;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Debug Wheels by the Individual")
public class DebugWheelsIndividual extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get("FL");
        DcMotor fr = hardwareMap.dcMotor.get("FR");
        DcMotor bl = hardwareMap.dcMotor.get("BL");
        DcMotor br = hardwareMap.dcMotor.get("BR");

        waitForStart();

        boolean up, down, left, right;
        while (opModeIsActive()) {
            up = gamepad1.dpad_up;
            down = gamepad1.dpad_down;
            left = gamepad1.dpad_left;
            right = gamepad1.dpad_right;

            if (up) {
                fl.setPower(0.5);
            } else {
                fl.setPower(0);
            }

            if (down) {
                fr.setPower(0.5);
            } else {
                fr.setPower(0);
            }

            if (left) {
                bl.setPower(0.5);
            } else {
                bl.setPower(0);
            }

            if (right) {
                br.setPower(0.5);
            } else {
                br.setPower(0);
            }
        }
    }
}
