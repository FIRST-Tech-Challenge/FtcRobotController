package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.SciLift;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Collect;
import org.firstinspires.ftc.teamcode.Flick;
import org.firstinspires.ftc.teamcode.FlickJr;
import org.firstinspires.ftc.teamcode.Foundation;
import org.firstinspires.ftc.teamcode.Gearbox;
import org.firstinspires.ftc.teamcode.Claw;

@TeleOp(name="15317 Teleop", group="Linear Opmode")

public class Teleop15317 extends LinearOpMode {

    //creating objects for all of the different parts
    private Drive d;
    private SciLift lift;
    // private Collect collector;
    private Claw claw;
    // private Arm arm;
    // private Flick flick;
    // private FlickJr flickjr;
    private Foundation foundation;
    private Gearbox gearbox;

            // y is down, controls the claw servo
    private boolean yIsDown = false;

    // y1 is down controls the gearbox servo
    private boolean y1IsDown = false;

    @Override
    public void runOpMode() {

        //initializing every motor, servo, and sensor
        //these names all need to match the names in the config
        d = new Drive(
                hardwareMap.get(DcMotor.class, "rbmotor"),
                hardwareMap.get(DcMotor.class, "rfmotor"),
                hardwareMap.get(DcMotor.class, "lfmotor"),
                hardwareMap.get(DcMotor.class, "lbmotor")
        );
        lift = new SciLift(
                hardwareMap.get(DcMotor.class, "liftmotor")
        );
        // arm = new Arm(
        //   hardwareMap.get(DcMotor.class, "armmotor")
        // );
        // collector = new Collect(
        //   hardwareMap.get(DcMotor.class, "col_left"),
        //   hardwareMap.get(DcMotor.class, "col_right"),
        //   hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor")
        // );
        // claw = new Claw(
        //   hardwareMap.get(Servo.class, "clawleft"),
        //   hardwareMap.get(Servo.class, "clawright")
        // );
        // flick = new Flick(
        //   hardwareMap.get(Servo.class, "flick")
        // );
        // flickjr = new FlickJr(
        //   hardwareMap.get(Servo.class, "hit")
        // );
        foundation = new Foundation(
                hardwareMap.get(Servo.class, "foundation")

        );
        gearbox = new Gearbox(
                hardwareMap.get(Servo.class, "gearbox")

        );

        waitForStart();
        while (opModeIsActive()) {
            //gamepad 1
            //drive..........sticks
            //turbo..........right trigger

            //gamepad 2
            //lift...........right stick
            //arm............left stick
            //foundation.....Y to grab, X to release
            //collection.....bumpers, left for out, right for in
            //flicks.........hold X
            //claw...........A to grab, B to release

            // gamepad 1
            d.setPower(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger
            );

            if (gamepad1.b && !y1IsDown) {
                y1IsDown = true;
                gearbox.nextPos();
            } else if (!gamepad1.b) {
                y1IsDown = false;
            }


            //gamepad 2
            if (gamepad2.y && !yIsDown) {
                yIsDown = true;
                foundation.nextPos();
            } else if (!gamepad2.y) {
                yIsDown = false;
            }

            // if (gamepad2.b) {
            //   claw.release();
            // } else if (gamepad2.a) {
            //   claw.grab();
            // }

            // if (gamepad2.x) { //the full flick, flickjr, claw process
            //   flick.setPos(0.23);
            //   if (flick.getPos() < 0.27) {
            //     flickjr.setPos(0.2);
            //     if (flickjr.getPos() < 0.25) {
            //       claw.grab();
            //     }
            //   }
            // } else {
            //   flick.down();
            //   flickjr.down();
            // }

            // if (gamepad2.right_bumper) {
            //   collector.in();
            // } else if (gamepad2.left_bumper) {
            //   collector.out();
            // } else {
            //   collector.rest();
            // }

            // if (gamepad2.left_stick_y > 0) {
            //     arm.extend(Math.abs(gamepad2.left_stick_y*0.75));
            // } else if (gamepad2.left_stick_y < 0){
            //     arm.retract(Math.abs(gamepad2.left_stick_y));
            // } else {
            //     arm.rest();
            // }

            if (-gamepad2.right_stick_y > 0) {
                lift.up(Math.abs(gamepad2.right_stick_y));
            } else if (-gamepad2.right_stick_y < 0){
                lift.down(Math.abs(gamepad2.right_stick_y));
            } else {
                lift.rest();
            }

            telemetry.addData("Status", "Run Time: ");
            // telemetry.addData("Collect Power", collector.getPower());
            // telemetry.addData("Dist Sensor", collector.getDistance());
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.addData("lf", d.getPowerlf());
            telemetry.addData("lb", d.getPowerlb());
            telemetry.addData("rf", d.getPowerrf());
            telemetry.addData("rb", d.getPowerrb());
            telemetry.addData("Clicks: ", d.getClickslf());
            telemetry.addData("Lift", lift.getClicks());
            telemetry.addData("gearbox", gearbox.getPos());
            //telemetry.addData("flickpos", flick.getPos());
            telemetry.update();

        }
    }
}
