// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.teamcode.Gpsbrain;
// import org.firstinspires.ftc.teamcode.Drive;
// import org.firstinspires.ftc.teamcode.Collect;
// import org.firstinspires.ftc.teamcode.Find;
// import org.firstinspires.ftc.teamcode.SciLift;
// import org.firstinspires.ftc.teamcode.Foundation;

// @Autonomous

// public class Autonomous15317 extends LinearOpMode {

//     private Drive d;
//     private Collect c;
//     private BNO055IMU imu;
//     private Gpsbrain gps;
//     private Find f;
//     private SciLift lift;
//     private Foundation foundation;

//     @Override
//     public void runOpMode() {

//         d = new Drive(
//           hardwareMap.get(DcMotor.class, "rbmotor"),
//           hardwareMap.get(DcMotor.class, "rfmotor"),
//           hardwareMap.get(DcMotor.class, "lfmotor"),
//           hardwareMap.get(DcMotor.class, "lbmotor")
//         );

//         c = new Collect(
//         hardwareMap.get(DcMotor.class, "col_left"),
//         hardwareMap.get(DcMotor.class, "col_right"),
//         hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor")
//         );

//         f = new Find();

//         lift = new SciLift(
//             hardwareMap.get(DcMotor.class, "liftmotor")
//         );

//         imu = hardwareMap.get(BNO055IMU.class, "imu");

//         gps = new Gpsbrain(d, imu, c, f, lift);
        
//         foundation = new Foundation(
//             hardwareMap.get(Servo.class, "foundation")
//           );

//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         waitForStart();

//         while (opModeIsActive()) {
//             gps.update(); //all the autonomous stuff
//             foundation.release();
//             if (!gps.turning) { gps.correct(); } //keeping the robot straight

//             telemetry.addData("State", gps.states[gps.count]);
//             telemetry.addData("Count", gps.count);
//             telemetry.addData("Angle", gps.seekAngle);
//             telemetry.addData("clicks", gps.d.getClickslf());
//             telemetry.addData("globalx", gps.globalx);
//             telemetry.addData("globaly", gps.globaly);
//             telemetry.addData("globala", gps.globala);
//             telemetry.addData("goalclicks", gps.goalclicks);
//             telemetry.addData("globaly", gps.relativey);
//             telemetry.update();

//         }
//     }
// }
