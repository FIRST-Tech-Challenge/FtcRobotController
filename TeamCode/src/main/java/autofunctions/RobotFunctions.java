package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

import java.util.ArrayList;

import global.TerraBot;
import util.CodeSeg;
import util.Rect;

import java.util.Random;
///////////////////////////////////////////////////////////////////////////////////import autoUtil.TerraCV.stoneP;


public class RobotFunctions {

    TerraBot bot = null;
    public LinearOpMode op = null;

    public TerraCV.RingNum ringnum;

    //Inspirational Messages
    TerraCV terraCV = new TerraCV();

    public ArrayList<String> ims = new ArrayList<>();

//    public int outtakeInd = 0;
//    public int lastOuttakeInd = 0;

    ElapsedTime timer = new ElapsedTime();

    public void init(TerraBot t, LinearOpMode o) {
        bot = t;
        op = o;
        initIMs();
        terraCV.init(o,false);
    }

    public void generateRandomIM(){
        Random rand = new Random();
        int index = rand.nextInt(ims.size());
        String out = ims.get(index+1);
        telemetryText(out);
    }

    public CodeSeg intake(final double pow){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.intake(pow);
            }
        };
    }

    public CodeSeg outtake(final double speed){
        return new CodeSeg() {
            @Override
            public void run() {
                double sp = speed*bot.MAX_OUTTAKE_SPEED;
                bot.outrController.setTargetSpeed(sp);
                bot.outlController.setTargetSpeed(sp);
                double outrPow = bot.outrController.getPow();
                double outlPow = bot.outlController.getPow();
                bot.outr.setPower(outrPow);
                bot.outl.setPower(outlPow);
            }
        };
    }

    public CodeSeg wobbleArm(final double deg, final double pow){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.arm.setTargetPosition(bot.degreesToTicks(deg));
                bot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.arm.setPower(pow);
                while (op.opModeIsActive() && bot.arm.isBusy()){ }
                bot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bot.arm.setPower(0);

            }
        };
    }

    public CodeSeg turnArm(final double pos){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.turnWobbleArm(pos);

            }
        };
    }
    public CodeSeg lift(final int index){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.lift(bot.liftControl.getPos(index));
            }
        };
    }
    public CodeSeg grab(final int index){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.grab(bot.grabControl.getPos(index));
            }
        };
    }
    public void scanRings(){
        while (!op.isStarted() && !op.isStopRequested()) {
            terraCV.takePictureBeforeInit();
            Rect cropped = new Rect(0, 0, 1280, 720).crop(610,450,490,100);
            TerraCV.RingNum num = terraCV.getRingNum(cropped);
            if(num != null) {
                telemetryText(num.toString());
                ringnum = num;
            }
        }
    }

    public CodeSeg shoot(final double speed){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.shooter.start();
                while (bot.shooter.executing){
                    bot.update();
                    bot.outtakeWithEncoders(speed);
                }
                if(!bot.shooter.pausing){
                    bot.outtake(0);
                }

            }
        };
    }

    public CodeSeg changeAcc(final double sx, final double sy, final double sh, final Path path){
        return new CodeSeg() {
            @Override
            public void run() {
                path.HAcc = path.HAccS*sh;
                path.XAcc = path.XAccS*sx;
                path.YAcc = path.YAccS*sy;
            }
        };
    }
    public CodeSeg changeKs(final double sk, final Path path){
        return new CodeSeg() {
            @Override
            public void run() {
                path.kScale = sk;
            }
        };
    }

//    public CodeSeg shoot2(final double speed){
//        return new CodeSeg() {
//            @Override
//            public void run() {
//                lift(1).run();
//                while (outtakeInd < 3){
//                    if(lastOuttakeInd != outtakeInd){
//                        timer.reset();
//                        shootControl(3).run();
//                        while (timer.seconds() < 0.3){}
//                        shootControl(2).run();
//                        lastOuttakeInd = outtakeInd;
//                    }
//                    bot.outtakeWithEncoders(speed);
//                }
//            }
//        };
//    }

//    public CodeSeg outNext(){
//        return new CodeSeg() {
//            @Override
//            public void run() {
//                outtakeInd++;
//            }
//        };
//    }

    public CodeSeg toggleOuttake(final TerraBot bot){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.outtaking = !bot.outtaking;
            }
        };
    }


    public CodeSeg shootControl(final int cur){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.shoot(bot.shootControlR.getPos(cur), bot.shootControlL.getPos(cur));
            }
        };
    }

    public CodeSeg updateXWithDis(final double x){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.odometry.tx = bot.odometry.cmToTicks(bot.getDisL2()-x);
            }
        };
    }

    public CodeSeg changeOuttakePow(final double pow, final double vs, final Path path){
        return new CodeSeg() {
            @Override
            public void run() {
                path.shootSpeed = pow;
                bot.outrController.setStartPow(bot.outtakeStartR*pow * vs);
                bot.outlController.setStartPow(bot.outtakeStartL*pow * vs);
            }
        };
    }




//    public void scanStonesBeforeInit(TerraCV cv){
//        while (!op.isStarted()){
//            cv.takePictureBeforeInit();
//            Rect area = new Rect(240, 400, 720, 150);
//
//            TerraCV.StonePos sp = cv.getStonePos(area);
//            if(sp != null) {
//                stonePos = sp;
//            }
//            if(stonePos.equals(TerraCV.StonePos.RIGHT)) {
//                op.telemetry.addData("status", "READy");
//            }else if(stonePos.equals(TerraCV.StonePos.MIDDLE)){
//                op.telemetry.addData("status", "REaDY");
//            }else if (stonePos.equals(TerraCV.StonePos.LEFT)){
//                op.telemetry.addData("status", "rEADY");
//            }
//            op.telemetry.update();
//        }
//    }
//
//    public void scanStones(Path p, final TerraCV cv){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                cv.takePicture();
//                Rect area = new Rect(0,0,1270, 710);
//                stonePos = cv.getStonePos(area);
//                stonePos = TerraCV.StonePos.RIGHT;
////                op.telemetry.addData("Stone", stonePos.toString());
////                op.telemetry.update();
//            }
//        });
//    }
//

//
//    public void grabFoundation(Path p, final double pos){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                bot.foundationGrab(pos);
//            }
//        });
//    }
//
//    public void flip(Path p, final double p1, final double p2){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                bot.flip(p1,p2);
//            }
//        });
//    }
//
//    public void grab(Path p, final double pos){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                bot.grab(pos);
//                //op.sleep(500);
//            }
//        });
//    }
//
//    public void pause(Path p, final long time){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                op.sleep(time);
//            }
//        });
//    }
//
//    public void resetOdometry(final Path p){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                odometry.reset();
//            }
//        });
//    }
//    public void resetHeading(final Path p, final int heading){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                odometry.theta = (bot.getHeading() - heading);
//            }
//        });
//    }
//
//    public void setAccuracy(final Path p, final double x, final double y, final double h){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                p.HACCURACY = h;
//                p.XACCURACY = x;
//                p.YACCURACY = y;
//            }
//        });
//    }
//    public void multiplyD(final Path p, final double scale){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                p.scaleD = scale;
//            }
//        });
//    }
//    public void setScale(final Path p,final double scale){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                p.multiplyKD(scale);
//            }
//        });
//
//    }
//
//    public void move(final double y, final double x, final double t, final double time){
//        timer.reset();
//        bot.move(y, x, t);
//        while (op.opModeIsActive() && timer.seconds() < time){}
//    }
//
//    public void customThread(final Path p, final CodeSeg code){
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                Thread t = new Thread(new Runnable() {
//                    @Override
//                    public void run() {
//                        code.run();
//                    }
//                });
//                t.start();
//            }
//        });
//    }

    public void telemetryText(final String text) {
        op.telemetry.addData(":", text);
        op.telemetry.update();
    }

//    public void telemetryValue(Path p, final String cap, final double d) {
//        p.addCustom(new CodeSeg() {
//            @Override
//            public void run() {
//                op.telemetry.addData(cap, d);
//                op.telemetry.update();
//            }
//        });
//    }


    public void initIMs(){
        ims.add("NitDaWit Just a Piece of ...");
        ims.add("The red is blood the person is you Im done with this so f u.");
        ims.add("No Excuses");
        ims.add("The tape is stuck!");
        ims.add("I can't do relic!");
        ims.add("Is Shuhul hurt");
        ims.add("Jeff is watching");
        ims.add("Bob Smith");
        ims.add("Robo Avatars crying");
        ims.add("Shuhuls computer");
        ims.add("Big Boy Roy");
        ims.add("Less Friction");
        ims.add("Your PC ran into fatal errors");
        ims.add("For testing purposes only");
        ims.add("My hands I messed up");
        ims.add("I made a mistake");
        ims.add("Flow Chart");
        ims.add("Ayush Servo");
        ims.add("Titanium dik");
        ims.add("Roy 'I need you for support' Help me up");
        ims.add("Peter and Steven have blessed you");
        ims.add("SEVEN STONE AUTO INIT");
        ims.add("Mogli");
        ims.add("Chipmunk");
        ims.add("FLippin leave");
        ims.add("UWU");
        ims.add("Jeff stop looking at this or Ill personally disqualify");
        ims.add("Safety glasses in the pits or GTFO");
        ims.add("Panda Chicken");
        ims.add("Bracket Sources");
        ims.add("'My Heart... its beating too fast I cant stop'");
        ims.add("14 volt big boy");
        ims.add("rolling sky grind");
        ims.add("this was supposed to be MY day");
        ims.add("1115, 8802, 14525, world record no penalties");
        ims.add("Higuchi nada nada bada enchilada na ana na...");

    }

    public void pauseBeforeInit(double secs){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (!op.isStarted() && time.seconds() < secs){}
    }
    public void pause(double secs){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (op.opModeIsActive() && time.seconds() < secs){}
    }

}
