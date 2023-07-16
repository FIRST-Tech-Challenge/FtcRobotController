package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
@Disabled
@TeleOp(name = "LEDTest")

public class LEDTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        PwPRobot pwprobot = new PwPRobot(this, false);
        RFLEDStrip led = new RFLEDStrip();
        waitForStart();

        while (opModeIsActive()) {

            //basic colors

            led.red();

            sleep(2500);

            led.blue();

            sleep(2500);

            led.orange();

            sleep(2500);

            led.yellow();

            sleep(2500);

            led.gold();

            sleep(2500);

            led.white();

            sleep(2500);

            led.gray();

            sleep(2500);

            led.pink();

            sleep(2500);

            led.aqua();

            sleep(2500);

            led.green();

            sleep(2500);

            led.black();

            sleep(2500);

            led.confetti();

            sleep(2500);

            led.lime();

            sleep(2500);

            led.violet();

            sleep(2500);





            //compound colors

            led.bluegreen();

            sleep(2500);

            led.blueviolet();

            sleep(2500);

            led.lawngreen();

            sleep(2500);

            led.redorange();

            sleep(2500);

            led.skyblue();

            sleep(2500);



            //dark

            led.darkblue();

            sleep(2500);

            led.darkgray();

            sleep(2500);

            led.darkgreen();

            sleep(2500);

            led.darkred();

            sleep(2500);





            //breath

            led.breathblue();

            sleep(2500);

            led.breathgray();

            sleep(2500);

            led.breathred();

            sleep(2500);





            //bpm

            led.bpmforest();

            sleep(2500);

            led.bpmlava();

            sleep(2500);

            led.bpmocean();

            sleep(2500);

            led.bpmparty();

            sleep(2500);

            led.bpmrainbow();

            sleep(2500);





            //waves

            led.wavesforest();

            sleep(2500);

            led.waveslava();

            sleep(2500);

            led.wavesocean();

            sleep(2500);

            led.wavesparty();

            sleep(2500);

            led.wavesrainbow();

            sleep(2500);





            //cp

            led.cpbpm();

            sleep(2500);





            //cp1

            led.cp1breathfast();

            sleep(2500);

            led.cp1breathslow();

            sleep(2500);

            led.cp1endtoendblacktoblack();

            sleep(2500);

            led.cp1heartbeatfast();

            sleep(2500);

            led.cp1heartbeatmedium();

            sleep(2500);

            led.cp1hearbeatslow();

            sleep(2500);

            led.cp1larsonscanner();

            sleep(2500);

            led.cp1lightchase();

            sleep(2500);

            led.cp1shot();

            sleep(2500);

            led.cp1strobe();

            sleep(2500);





            //cp12

            led.cp12colorgradient();

            sleep(2500);

            led.cp12colorwaves();

            sleep(2500);

            led.cp12endtoendblend();

            sleep(2500);

            led.cp12sinelon();

            sleep(2500);

            led.cp12endtoendblend1to2();

            sleep(2500);

            led.cp12noblending();

            sleep(2500);

            led.cp12sparkle1on2();

            sleep(2500);

            led.cp12sparkle2on1();

            sleep(2500);

            led.cp12twinkles();

            sleep(2500);





            //cp2

            led.cp2breathfast();

            sleep(2500);

            led.cp2breathslow();

            sleep(2500);

            led.cp2endtoendblendtoblack();

            sleep(2500);

            led.cp2heartbeatfast();

            sleep(2500);

            led.cp2hearbeatmedium();

            sleep(2500);

            led.cp2heartbeatslow();

            sleep(2500);

            led.cp2larsonscanner();

            sleep(2500);

            led.cp2lightchase();

            sleep(2500);

            led.cp2();

            sleep(2500);

            led.cp2strobe();

            sleep(2500);





            //fire

            led.fire();

            sleep(2500);

            led.firelarge();

            sleep(2500);

            led.firemedium();

            sleep(2500);





            //heartbeat

            led.heartbeatblue();

            sleep(2500);

            led.heartbeatgray();

            sleep(2500);

            led.heartbeatred();

            sleep(2500);

            led.heartbeatwhite();

            sleep(2500);





            //larson scanner

            led.larsonscannergray();

            sleep(2500);

            led.larsonscannerred();

            sleep(2500);





            //light chase

            led.lightchaseblue();

            sleep(2500);

            led.lightchasegray();

            sleep(2500);

            led.lightchasered();

            sleep(2500);





            //rainbow

            led.rainbow();

            sleep(2500);

            led.rainbowforest();

            sleep(2500);

            led.rainbowrainbow();

            sleep(2500);

            led.rainbowlava();

            sleep(2500);

            led.rainbowocean();

            sleep(2500);

            led.rainbowparty();

            sleep(2500);

            led.rainbowwithglitter();

            sleep(2500);





            //shot

            led.shotblue();

            sleep(2500);

            led.shotred();

            sleep(2500);

            led.shotwhite();

            sleep(2500);





            //sinelon

            led.sinelonforest();

            sleep(2500);

            led.sinelonlava();

            sleep(2500);

            led.sinelonocean();

            sleep(2500);

            led.sinelonparty();

            sleep(2500);

            led.sinelonrainbow();

            sleep(2500);





            //strobe

            led.strobeblue();

            sleep(2500);

            led.strobegold();

            sleep(2500);

            led.strobered();

            sleep(2500);

            led.strobewhite();

            sleep(2500);





            //twinkles

            led.twinklesforest();

            sleep(2500);

            led.twinkleslava();

            sleep(2500);

            led.twinklesocean();

            sleep(2500);

            led.twinklesparty();

            sleep(2500);

            led.twinklesrainbow();

            sleep(2500);
        }
        stop();
    }
}
