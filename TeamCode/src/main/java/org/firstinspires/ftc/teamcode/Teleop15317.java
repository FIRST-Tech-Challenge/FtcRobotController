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
// import org.firstinspires.ftc.teamcode.Foundation;
import org.firstinspires.ftc.teamcode.TwoPosServo;
import org.firstinspires.ftc.teamcode.Claw;

@TeleOp(name="15317 Teleop", group="Linear Opmode")

public class Teleop15317 extends LinearOpMode {

    //creating objects for all of the different parts
    private Drive d;
    private SciLift lift;
//    private Collect collector;
    private Intake intake;
//    private HopperWheel hopperwheel;
    // private Claw claw;
    // private Arm arm;
    // private Flick flick;
    // private FlickJr flickjr;
    private TwoPosServo claw; //the file this used to be is still called Foundation btw
   // private TwoPosServo gearbox;
    private boolean clawButtonIsDown = false; // controls the claw servo button press
    private boolean gearboxButtonIsDown = false; // controls the gearbox servo button press


    private Servo gear;
    private double gearmax = 0.6; // Maximum rotational position
    private double gearmin = 0.5; // Minimum rotational position
    private String gearcurrentPos = "min";

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
//         collector = new Collect(
//           hardwareMap.get(DcMotor.class, "col_left"),
//           hardwareMap.get(DcMotor.class, "col_right"),
//           hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor")
//         );
        intake = new Intake(
           hardwareMap.get(DcMotor.class, "topmotor"),
           hardwareMap.get(DcMotor.class, "botmotor")
        );
//        hopperwheel = new HopperWheel(
//                hardwareMap.get(DcMotor.class, "hopper"),
//                hardwareMap.get(DcMotor.class, "wheel")
//        );
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
        claw = new TwoPosServo(
            hardwareMap.get(Servo.class, "claw"),
            0.5, 1.0
        );

        gear = hardwareMap.get(Servo.class, "gearbox");

        waitForStart();
        while (opModeIsActive()) {
            //gamepad 1
            //drive..........sticks
            //gearbox........b button
            //claw...........y button

            //gamepad 2
            //lift...........right stick up and down


            // gamepad 1
            d.setPower(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.right_trigger
            );

            if (gamepad1.b && !gearboxButtonIsDown) {
                gearboxButtonIsDown = true;
                if(gearcurrentPos.equals("min")) {
                    gearcurrentPos = "max";
                    gear.setPosition(gearmax);
                } else if(gearcurrentPos.equals("max")) {
                    gearcurrentPos = "min";
                    gear.setPosition(gearmin);
                }
            } else if (!gamepad1.b) {
                gearboxButtonIsDown = false;
            }



            if (gamepad1.y && !clawButtonIsDown) {
                clawButtonIsDown = true;
                claw.nextPos();
            } else if (!gamepad1.y) {
                clawButtonIsDown = false;
            }


            //gamepad 2


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

//             if (gamepad2.right_bumper) {
//               collector.in();
//             } else if (gamepad2.left_bumper) {
//               collector.out();
//             } else {
//               collector.rest();
//             }

            if (gamepad2.right_bumper) {
               intake.in();
             } else if (gamepad2.left_bumper) {
               intake.out();
             } else {
               intake.rest();
             }

//            if (gamepad2.left_bumper) {
//               hopperwheel.out();
//             } else {
//               hopperwheel.rest();
//             }

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
//            telemetry.addData("Collect Power", collector.getPower());
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
         //   telemetry.addData("gearbox", gear.getPos());
            telemetry.addData("gearbox", claw.getPos());
            //telemetry.addData("flickpos", flick.getPos());
            telemetry.update();

        }
    }
}
