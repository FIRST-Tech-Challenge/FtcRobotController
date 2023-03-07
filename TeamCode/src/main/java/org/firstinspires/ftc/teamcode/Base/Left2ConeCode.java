//package org.firstinspires.ftc.teamcode.Base;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class Left2ConeCode extends LinearOpMode {
//
//
//        MainBase base = new MainBase();
//        VariablesBase var = new VariablesBase();
//
//        public void Left2Cone() {
//
//
//                //Cone 1
//                base.Arezoo(var.DRIVE_SPEED07, 30, 30, 30, 30, 0, this);
//                base.navxgyroStrafe(var.DRIVE_SPEED06, -10, -10, -10, -10, 0, this);
//                base.navxgyroTurn(var.TURN_SPEED03, 90, this);
////        telemetry.addData("YAW:", base.navx_device.getYaw());
////        telemetry.update();
////        sleep(5000);
//                base.navxgyroHold(var.GYRO_HOLD_SPEED, 91, var.GYRO_HOLD_TIME, this);
//                base.Arezoo(var.DRIVE_SPEED07, -25, -25, -25, -25, 91, this);
//                base.goLift(30, .6);
//                base.Arezoo(var.DRIVE_SPEED07, -25, -25, -25, -25, 90, this);
//                base.navxgyroTurn(var.TURN_SPEED03, 44, this);
//                base.navxgyroHold(.05, 41, 1.2, this);
//                base.Arezoo(var.DRIVE_SPEED07, -12.6, -12.6, -12.6, -12.6, 41, this);
//                base.letGoGirl(this);
//
//                //Cone 2
//                base.Arezoo(var.DRIVE_SPEED07, 16.8, 16.8, 16.8, 16.8, 41, this);
//                base.navxgyroTurn(var.TURN_SPEED03, -2, this);
//                base.navxgyroHold(var.GYRO_HOLD_SPEED, -4, var.GYRO_HOLD_TIME, this);
//                base.goLift(-19, .6);
//                base.Arezoo(var.DRIVE_SPEED04, -10, -10, -10, -10, -4, this);
//                base.Arezoo(var.DRIVE_SPEED07, -53, -54, -54, -54, -4, this);
//                base.findLine(var.FIND_LINE_TIME, this);
//                //Grab Cone 2
//                base.goLift(-6, .7);
//                base.getItGirl(this);
//                base.goLift(26, .8);
//                base.Arezoo(var.DRIVE_SPEED07, 28, 28, 28, 28, -1, this);
//                base.navxgyroTurn(.5, 124, this);
//                base.navxgyroHold(.05, 140, 1.2, this);
//                base.Arezoo(var.DRIVE_SPEED07, -9.8, -9.8, -9.8, -9.8, 140, this);
//                base.letGoGirl(this);
//        }
//
//        @Override
//        public void runOpMode() throws InterruptedException {
//
//        }
//}
