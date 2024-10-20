//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//public class ArmObsolete {
//    private Robot robot;
//    private Gamepad gamepad;
//    private Servo servo;
//    double speed_factor = 1.0;
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//public class ArmObsolete {
//    private Robot robot;
//    private Gamepad gamepad;
//    private Servo servo;
//    double speed_factor = 1.0;
//
//    private boolean verbose = false;
//
//    public ArmObsolete(Robot robot, Gamepad gamepad)
//    {
//        this.servo = robot.servoArm;
//        this.robot = robot;
//        this.gamepad = gamepad;
//    }
//    public void auton() {
//        //autoOp(pos_auton);
//    }
//
//    public void fold() {
//       // autoOp(pos_min);
//    }
//
//    public void arm_pixel()
//    {
//       // robot.servoArm.setPosition(pos_pixel);
//    }
//
//    public void arm_fold()
//    {
//        //robot.servoArm.setPosition(pos_folded);
//    }
//
//    public void arm_backdrop() {
//        //robot.servoArm.setPosition(pos_backdrop);
//    }
//
//    public void arm_whitepixel() {
//        //robot.servoArm.setPosition(pos_whitepixel);
//    }
//
//    public void operate() {
//        //autoOpCompletionCheck();
//        if (gamepad.dpad_down) {
//            // Go to the bottom
//            //autoOp(pos_min);
//        } else if (gamepad.dpad_up) {
//            // Goto the middle
//            //autoOp(pos_middle);
//        } else if (gamepad.right_stick_y != 0) {
//            this.servo.setPower(-gamepad.right_stick_y * speed_factor);
//        } else {
//           this.servo.setPower(0);
//        }
//        //logUpdate();
//    }
//}
//    private boolean verbose = false;
//
//    public ArmObsolete(Robot robot, Gamepad gamepad)
//    {
//        this.servo = robot.servoArm;
//        this.robot = robot;
//        this.gamepad = gamepad;
//    }
//    public void auton() {
//        //autoOp(pos_auton);
//    }
//
//    public void fold() {
//       // autoOp(pos_min);
//    }
//
//    public void arm_pixel()
//    {
//       // robot.servoArm.setPosition(pos_pixel);
//    }
//
//    public void arm_fold()
//    {
//        //robot.servoArm.setPosition(pos_folded);
//    }
//
//    public void arm_backdrop() {
//        //robot.servoArm.setPosition(pos_backdrop);
//    }
//
//    public void arm_whitepixel() {
//        //robot.servoArm.setPosition(pos_whitepixel);
//    }
//
//    public void operate() {
//        //autoOpCompletionCheck();
//        if (gamepad.dpad_down) {
//            // Go to the bottom
//            //autoOp(pos_min);
//        } else if (gamepad.dpad_up) {
//            // Goto the middle
//            //autoOp(pos_middle);
//        } else if (gamepad.right_stick_y != 0) {
//            this.servo.setPower(-gamepad.right_stick_y * speed_factor);
//        } else {
//           this.servo.setPower(0);
//        }
//        //logUpdate();
//    }
//}

