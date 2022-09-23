package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.PixyCam;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class PixySystem2 {
    private final int YELLOW_SIGNATURE = 3;
    private final int WHITE_SIGNATURE = 4;
    private Telemetry telemetry;
    private PixyCam pixy;
    private int yellowPos;
    private int white1Pos;
    private int white2Pos;

    public PixySystem2 (OpMode opMode) {
        pixy = hardwareMap.get(PixyCam.class, "PixyCam");
        pixy.setBlockCount(3);
    }

    public void runPixySystem() {
        telemetry.addLine("running PixySystem");
        printValues();
        centerYellow();
    }

    public void printValues() {
        telemetry.addData("yellow: ", yellowPos);
        telemetry.addData("white1: ", white1Pos);
        telemetry.addData("white2: ", white2Pos);
    }

    private void getValues() {
        int yellowCount = -1;
        int yellowSum = 0;
        int white1Count = -1;
        int white1Sum = 0;
        int white2Count = -1;
        int white2Sum = 0;

        for (int i = 0; i < 3; i++) {
            int whiteVal1 = 0;
            int whiteVal2 = 0;
            ArrayList<PixyCam.Block> blocks = pixy.getBlocks();
            for (PixyCam.Block block : blocks) {
                telemetry.addLine("sig: " + block.signature + "   " + "xCenter: " + block.xCenter);
                if (block.signature == YELLOW_SIGNATURE) {
                    yellowCount++;
                    yellowSum += block.xCenter;
                } else if (block.signature == WHITE_SIGNATURE) {
                    if (whiteVal1 == 0) {
                        whiteVal1 = block.xCenter;
                    } else {
                        whiteVal2 = block.xCenter;
                    }
                }

                if (whiteVal1 > whiteVal2 && whiteVal2 != 0) {
                    white1Sum += whiteVal2;
                    white1Count++;
                    white2Sum = whiteVal1;
                    white2Count++;
                }
            }
        }

        yellowPos = yellowSum / yellowCount;
        white1Pos = white1Sum / white1Count;
        white2Pos = white2Sum / white2Count;

        printValues();
    }

    public void centerYellow() {
        // if yellow is visible but not centered, center it
        if(yellowPos > 0) {
            telemetry.addLine("centering yellow -- yellow is visible");
            while (yellowPos < 255 / 2 - 5) {
                getValues();
            }
            while (yellowPos > 255 / 2 + 5) {
                getValues();
            }

        }
        // if yellow isnt visible but both white are and they're next to each other
        else if(white2Pos != 0 && white1Pos != 0 && white2Pos - white1Pos < 100) {
            telemetry.addLine("centering yellow -- white next to each other");
            if(white2Pos < 2 * 255 / 3) {
                while(white2Pos > 255 / 3) {
                    getValues();
                }
            } else if(white1Pos > 255 / 3) {
                while(white1Pos < 2 * 255 / 3) {
                    getValues();
                }
            }
        }
        // if yellow isnt visible but both white are and they're not next to each other
        else if (white2Pos - white1Pos >= 100) {
            telemetry.addLine("centering yellow -- yellow in middle");
            while (white1Pos > 255 - white2Pos) {
                getValues();
            }
            while (white1Pos < 255 - white2Pos) {
                getValues();
            }
        }
    }

}