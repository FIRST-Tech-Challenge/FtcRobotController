package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Zayne;//package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Zayne;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Basic_Drive_Files.OLD_AND_NEW_IMU.BOTHMecanum;
//
//public class LearnComandOpZanye {
//
//
//    //@Disabled
//
//    //@TeleOp
///*
//    public class LearnCommandOpZanye extends CommandOpMode {
//        // what this is doing is making it move in any direction. Left Right Up. U name it. EWHY AM I TYPING THIS
//        BOTHMecanum drive;
//
//        private DcMotor[] motors; // duhhhh
//
//
//       // @Override
//=======
//
//
//
//    @Disabled
//
//    @TeleOp
//    public abstract class LearnComandOpZanye extends CommandOpMode{ //QHY IS IT NOT STAYING AS NOT ABSTRACTTTTTTT
//        // what this is doing is making it move in any direction. Left Right Up. U name it. EWHY AM I TYPING THIS
//        BOTHMecanum drive;  // make your own drive later
//
//        private DcMotor[] motors; // duhhhh
//        int power = 1;
//
//
//       // @Override
//        //@Override
//>>>>>>> be5580a474cd49795f42b95c4c93c4a6343107c2
//
//        public void initalize() {
//
//            GamepadEx mech = new GamepadEx(gamepad2);//Gamepad 2 should be replaced with whatever gamepad you want. GamepadEx is just making the gamepad class
//
//            motors = new DcMotor[]{
//                    hardwareMap.dcMotor.get("fl"),
//                    hardwareMap.dcMotor.get("fr"),
//                    hardwareMap.dcMotor.get("bl"),
//                    hardwareMap.dcMotor.get("br"),
//
//            };
//
//<<<<<<< HEAD
//            learnComandOpZanye imu = hardwareMap.get(learnComandOpZanye.class, "imu"); // This code is just initializing the hardware map
//            //imu = (LearnComandOpZanye) hardwareMap.get(IMU.class, "imu"); IDK what this does yet sooo. @Justinn
//
//            drive = new BOTHMecanum(motors, (IMU) imu);
//
//
//
//        }
//    }
//
//*/
//}
//            LearnComandOpZanye imu = hardwareMap.get(LearnComandOpZanye.class, "imu"); // This code is just initializing the hardware map
//            imu = (LearnComandOpZanye) hardwareMap.get(IMU.class, "imu");// IDK what this does yet sooo. We Chilling
//
//            drive = new BOTHMecanum(motors, (IMU) imu); // drivebase urself
//            //telemetry.addLine("Done"); Not really needed except for when actually testing
//            //telemetry.update();
//
//        }
//        @Override
//        public void run(){
//            super.run();
//            drive.OLDdrive(gamepad1.left_stick_y, gamepad1.left_stick_x * power, -gamepad1.right_stick_x ); // power can be changed to make it move faster based on the magnitude
//
//            if(gamepad1.left_bumper || gamepad1.right_bumper){ // This can be changed to make whatever button reset. Maybe... The middle button
//                drive.OLDreset();
//                gamepad1.rumble(250);//reseting the angle
//            }
//        }
//    }
//
//>>>>>>> be5580a474cd49795f42b95c4c93c4a6343107c2
