package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp(name="testing",group="teleop")
//it's called testing in the phone
public class Intake extends LinearOpMode{
    DcMotor one;
    DcMotor two;
//two motors on the intake, can get changed to any name that is needed

    boolean press=false;
    boolean rotate=false;

    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.dcMotor.get("one");
        two = hardwareMap.dcMotor.get("two");
        //adding motors into TeleOp
        waitForStart();


        ElapsedTime m = new ElapsedTime();
        m.reset();

        while (opModeIsActive()){

            driveMecanum(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            //depending on what buttons you want controlling the intake you can change it
            //But as from this program it moves when the joy sticks are moved
        }


    }
//the below is setting the power the motors
    private void setSpeeds(double oneSpeed, double twoSpeed) {
        double largest = 1;
        largest = Math.max(largest, Math.abs(oneSpeed));
        largest = Math.max(largest, Math.abs(twoSpeed));

        one.setPower(oneSpeed / largest);
        two.setPower(twoSpeed / largest);

    }

    void driveMecanum(double forward, double strafe, double rotate) {
        double one = forward + strafe + rotate;
        double two = forward + strafe + rotate;

        setSpeeds(one, two);
    }



}
