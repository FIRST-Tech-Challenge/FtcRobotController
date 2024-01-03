//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//@TeleOp(group = "drive")
//public class TeleOpPhase3 extends LinearOpMode {
//    // "Exponentially weighted moving average". This class can be used to create ramping for translations and heading.
//    // Essentially, the EWMA function creates returns an exponential curve given an jump
//    // Currently, we are using EWMA for our translation ramping.
//    class Ewma {
//        double mAlpha = 0;
//        double mLastValue = 0;
//        public Ewma(double alpha) {
//            mAlpha = alpha;
//        }
//
//        public double update(double x) {
//            mLastValue = mAlpha * x + (1 - mAlpha) * mLastValue;
//            return mLastValue;
//        }
//    }
//
//    private static double sigmoid(int x, double horiStretch, double horiOffset) {
//        return 1 / (1 + Math.exp(horiStretch * (-x + horiOffset)));
//    }
//
//    class Lift {
//        private DcMotorEx mLiftMotor;
//        private PIDFController mLiftPID;
//        private int mLiftStage0, mLiftStage1, mLiftStage2, mLiftStage3;
//        private int mLiftStage = 0;
//        private int mLiftSpeed = 90;
//
//        public Lift(DcMotorEx liftMotor, PIDFController liftPID, int liftStage0, int liftStage1, int liftStage2, int liftStage3, int liftSpeed) {
//            DcMotorEx mLiftMotor = liftMotor;
//            PIDFController mLiftPID = liftPID;
//            mLiftStage0 = liftStage0;
//            mLiftStage1 = liftStage1;
//            mLiftStage2 = liftStage2;
//            mLiftStage3 = liftStage3;
//            mLiftSpeed = liftSpeed;
//        }
//
//        public void setLiftStage(int x) {
//            mLiftStage = x;
//        }
//
//        public int getLiftStage() {
//            return mLiftStage;
//        }
//
//        public void goToLiftStage() {
//            // Corresponding each liftStage with a target position
//            switch (mLiftStage) {
//                case 0:
//                    mLiftPID.setTargetPosition(mLiftStage0);
//                    break;
//                case 1:
//                    mLiftPID.setTargetPosition(mLiftStage1);
//                    break;
//                case 2:
//                    mLiftPID.setTargetPosition(mLiftStage2);
//                    break;
//                case 3:
//                    mLiftPID.setTargetPosition(mLiftStage3);
//            }
//
//            // Making the liftMotor run to the designated position (measured in ticks)
//            mLiftMotor.setTargetPosition((int) mLiftPID.getTargetPosition());
//            mLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            // Set power of the liftMotor depending on how close it is to the target postion
//            mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//            int mLiftError = Math.abs((int) mLiftMotor.getCurrentPosition() - (int) mLiftPID.getTargetPosition());
//            mLiftMotor.setPower(sigmoid(mLiftError, 0.01, 250));
//        }
//
//        public void setLiftPositionManually(String direction) {
//            switch (direction) {
//                case "up":
//                    mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                    break;
//                case "down":
//                    mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                    break;
//            }
//
//            // Making the liftMotor run to the designated position, but this time, it's affected by the lift speed
//            mLiftPID.setTargetPosition(mLiftMotor.getCurrentPosition() + mLiftSpeed);
//            mLiftMotor.setPower(0.5);
//            mLiftMotor.setTargetPosition((int) mLiftPID.getTargetPosition());
//            }
//        }
//    }
//
//    class Arm {
//        private ServoImplEx mArmServo1;
//        private ServoImplEx mArmServo2;
//
//        public Arm(ServoImplEx armServo1, ServoImplEx armServo2) {
//            mArmServo1 = armServo1;
//            mArmServo2 = armServo2;
//        }
//
//
//
//
//    }
//
//
//}
