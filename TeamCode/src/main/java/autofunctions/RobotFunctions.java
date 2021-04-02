package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import global.TerraBot;
import util.CodeSeg;
import util.Rect;


public class RobotFunctions {

    TerraBot bot = null;
    public LinearOpMode op = null;

    public TerraCV.RingNum ringnum;

    TerraCV terraCV = new TerraCV();

    public void init(TerraBot t, LinearOpMode o) {
        bot = t;
        op = o;
        terraCV.init(o,false);
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

    public void telemetryText(final String text) {
        op.telemetry.addData(":", text);
        op.telemetry.update();
    }
}
