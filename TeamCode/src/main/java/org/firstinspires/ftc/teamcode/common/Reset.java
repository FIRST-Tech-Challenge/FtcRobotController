package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class Reset {
    HardwareDrive robot;
    GlobalPosSystem globalPosSystem;
    Constants constants = new Constants();
    ElapsedTime gapTime = new ElapsedTime();
    int waitForMS = 500;
    double prevTime=0;
    double currentTime=0;

    boolean STOP_RESET_L = false;
    boolean STOP_RESET_R = false;
    boolean isResetCycle = false;

    public Reset(HardwareDrive r, GlobalPosSystem gps){
        robot = r;
        globalPosSystem=gps;
        gapTime.reset();
    }

    public void reset(boolean shouldReset){
        if (shouldReset){
            if(gapTime.milliseconds()-prevTime>waitForMS){
                updateReset();
            }else{
                robot.botL.setPower(0);
                robot.topL.setPower(0);
                robot.botR.setPower(0);
                robot.topR.setPower(0);
            }

        } else{
            STOP_RESET_L = false;
            STOP_RESET_R = false;
            isResetCycle = false;
            gapTime.reset();
            prevTime=gapTime.milliseconds();
        }
    }

    private void updateReset(){
        int rotateL = (robot.topL.getCurrentPosition() + robot.botL.getCurrentPosition()) / 2; //total rotation of left module
        int rotateR = (robot.topR.getCurrentPosition() + robot.botR.getCurrentPosition()) / 2; //total rotation of right module
        //this won't work once you implement table-spinning.  Will probably need to use the GPS then.


        rotateL %= constants.CLICKS_PER_PURPLE_REV;
        rotateR %= constants.CLICKS_PER_PURPLE_REV;

        int topLTarget = (int)((robot.topL.getCurrentPosition() - rotateL));
        int botLTarget = (int)((robot.botL.getCurrentPosition() - rotateL));
        int topRTarget = (int)((robot.topR.getCurrentPosition() - rotateR));
        int botRTarget = (int)((robot.botR.getCurrentPosition() - rotateR));


        if (!isResetCycle){
            isResetCycle = true;

            robot.topL.setTargetPosition(topLTarget);
            robot.botL.setTargetPosition(botLTarget);
            robot.topR.setTargetPosition(topRTarget);
            robot.botR.setTargetPosition(botRTarget);

            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.botL.setPower(0.3);
            robot.topL.setPower(0.3);
            robot.botR.setPower(0.3);
            robot.topR.setPower(0.3);
        }

        else{
            if (robot.topL.getCurrentPosition() == topLTarget && robot.botL.getCurrentPosition() == botLTarget){
                robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.botL.setPower(0);
                robot.topL.setPower(0);

                globalPosSystem.hardResetGPS();

                STOP_RESET_L = true;
            } else if (!STOP_RESET_L){
                robot.botL.setPower(0.3);
                robot.topL.setPower(0.3);
            }

            if (robot.topR.getCurrentPosition() == topRTarget && robot.botR.getCurrentPosition() == botRTarget){
                robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.botR.setPower(0);
                robot.topR.setPower(0);

                globalPosSystem.hardResetGPS();

                STOP_RESET_R = true;
            } else if (!STOP_RESET_R){
                robot.botR.setPower(0.3);
                robot.topR.setPower(0.3);
            }
        }
    }
}
