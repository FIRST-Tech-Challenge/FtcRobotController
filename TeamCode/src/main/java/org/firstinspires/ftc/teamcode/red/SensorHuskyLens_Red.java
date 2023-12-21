/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.red;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blue.SensorHuskyLens_Blue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
public class SensorHuskyLens_Red {
    public static final int ID_BLUE = 1;
    public static final int ID_RED = 2;
    private static final int ID = ID_RED;

    private HuskyLens huskyLens;
    public enum TagDirection{
        UNKOWN,
        LEFT,
        MIDDLE,
        RIGHT,
    }

    private static final int BLOCK_NUM_DETECTED_1 = 1;
    private static final int BLOCK_NUM_DETECTED_3 = 3;
    private static final int BLOCK_NUM_DETECTED_4 = 4;
    private static final int MAX_BLOCK_DISTANCE = 100;
    private TagDirection tagDirection = TagDirection.UNKOWN;
    private Boolean bDetectedtag = false;

    protected HardwareMap hardwareMap = null;
    protected Telemetry telemetry;
    public void init(HardwareMap hw, Telemetry t){
        hardwareMap = hw;
        telemetry = t;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        initBeforeOpMode();
    }

    private void initBeforeOpMode(){
        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if(null == huskyLens){
            telemetry.addData(">>", "huskylens ERROR!!!");
            telemetry.update();
            return ;
        }

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public TagDirection calculateDirection(){

        /*
         * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
         * Block represents the outline of a recognized object along with its ID number.
         * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
         * referenced in the header comment above for more information on IDs and how to
         * assign them to objects.
         *
         * Returns an empty array if no objects are seen.
         */
        calculateDirection(huskyLens.blocks());
        telemetry.update();

        return tagDirection;
    }

    private ArrayList<HuskyLens.Block> getValidBlocks(HuskyLens.Block[] blocks){
        ArrayList<HuskyLens.Block> blocksArray = new ArrayList<HuskyLens.Block>();

        // 能识别的block有点少，先只根据ID过滤
        // 返回最合适的blocks
        for(int i = 0; i < blocks.length; ++i){
            //nDistance[i] = Math.sqrt();
            if(ID == blocks[i].id){
                blocksArray.add(blocks[i]);
            }
        }

//        //
//        //如果识别的blocks太多，可能是有误识别的情况
//        if(BLOCK_NUM_DETECTED_4 <= blocksArray.size()){
//            Collections.sort(blocksArray, new Comparator<HuskyLens.Block>() {
//                @Override
//                public int compare(HuskyLens.Block block1, HuskyLens.Block block2) {
//                    return Integer.compare(block1.y, block2.y);
//                }
//            });
//
//            int nDistance = blocksArray.get(1).y - blocksArray.get(0).y;
//            if(nDistance > MAX_BLOCK_DISTANCE){
//                blocksArray.remove(0);
//                telemetry.addData(">>>", "remove one detected block");
//                telemetry.update();
//            }
//        }

        return blocksArray;
    }

    public static double calculateDistance(int x1, int y1, int x2, int y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public static double calculateBlockDistance(HuskyLens.Block blockOne, HuskyLens.Block blockTwo) {
        return SensorHuskyLens_Red.calculateDistance(blockOne.x, blockOne.y, blockTwo.x, blockTwo.y);
    }

    //
    // 根据4个两两之间的距离来计算
    private Boolean cal4DetectedBlocksDirection(ArrayList<HuskyLens.Block> blocksArray){
        //1. 将blocks按照x的位置进行排序
        Collections.sort(blocksArray, new Comparator<HuskyLens.Block>() {
            @Override
            public int compare(HuskyLens.Block block1, HuskyLens.Block block2) {
                return Integer.compare(block1.x, block2.x);
            }
        });

        //2. 计算相邻的block之间的距离
        double nDistanceArray[] = new double[3];
        for (int i = 0; i + 1 < blocksArray.size(); i++) {
            nDistanceArray[i] = SensorHuskyLens_Red.calculateBlockDistance(blocksArray.get(i), blocksArray.get(i+1));
        }

        //3. 找到block之间的距离最小的索引
        int nIndex = 0;
        double minDistance = Arrays.stream(nDistanceArray)
                .min()
                .orElse(Double.MAX_VALUE);
        for (; nIndex < 3; nIndex++) {
            if(minDistance == nDistanceArray[nIndex]){
                break;
            }
        }

        //4. 根据block之间的距离来计算tag所放的位置
        if(0 == nIndex){
            tagDirection = TagDirection.LEFT;
        } else if(1 == nIndex){
            tagDirection = TagDirection.MIDDLE;
        } else if(2 == nIndex){
            tagDirection = TagDirection.RIGHT;
        }

        return true;
    }

    //
    // 根据面积来计算
    private Boolean cal3DetectedBlocksDirection(ArrayList<HuskyLens.Block> blocksArray){

        //1. 将blocks按照x的位置进行排序
        Collections.sort(blocksArray, new Comparator<HuskyLens.Block>() {
            @Override
            public int compare(HuskyLens.Block block1, HuskyLens.Block block2) {
                return Integer.compare(block1.x, block2.x);
            }
        });

        //2. 计算block的面积
        double nAreaArray[] = new double[3];
        for (int i = 0; i < blocksArray.size(); i++) {
            nAreaArray[i] = blocksArray.get(i).width * blocksArray.get(i).height;
        }

        //3. 找到block之间的面积最大的索引
        int nIndex = 0;
        double maxDistance = Arrays.stream(nAreaArray)
                .max()
                .orElse(Double.MIN_VALUE);
        for (; nIndex < 3; nIndex++) {
            if(maxDistance == nAreaArray[nIndex]){
                break;
            }
        }

        //4. 根据block的面积来计算tag所放的位置
        if(0 == nIndex){
            tagDirection = TagDirection.LEFT;
        } else if(1 == nIndex){
            tagDirection = TagDirection.MIDDLE;
        } else if(2 == nIndex){
            tagDirection = TagDirection.RIGHT;
        }

        return true;
    }


    public static final int LEFT_AREA_X = 80;
    public static final int RIGHT_AREA_X = 240;
    private Boolean calObjectedDetectedBlocksDirection(ArrayList<HuskyLens.Block> blocksArray){
        //
        HuskyLens.Block block = blocksArray.get(0);
        int nXPos = block.x;
        //4. 根据block的面积来计算tag所放的位置
        if(nXPos < LEFT_AREA_X){
            tagDirection = TagDirection.LEFT;
        } else if(LEFT_AREA_X <= nXPos && nXPos <= RIGHT_AREA_X){
            tagDirection = TagDirection.MIDDLE;
        } else if(nXPos > RIGHT_AREA_X){
            tagDirection = TagDirection.RIGHT;
        }

        return true;
    }


    private Boolean calBlocksDirection(ArrayList<HuskyLens.Block> blocksArray){
        //1. 计算block的面积
        double nAreaArray[] = new double[blocksArray.size()];
        for (int i = 0; i < blocksArray.size(); i++) {
            HuskyLens.Block block = blocksArray.get(i);
            int nXPos = block.x;
            double area = 0.0f;
            if(nXPos < LEFT_AREA_X){
                area = block.width * block.height;
                area -= 25.0 * 25.0;
            } else if(LEFT_AREA_X <= nXPos && nXPos <= RIGHT_AREA_X){
                area = block.width * block.height;
            } else if(nXPos > RIGHT_AREA_X){
                area = block.width * block.height;
                area -= 25.0 * 25.0;
            }
            nAreaArray[i] = area;
        }

        //2. 找到block之间的面积最大的索引
        int nIndex = 0;
        double maxDistance = Arrays.stream(nAreaArray)
                .max()
                .orElse(Double.MIN_VALUE);
        for (; nIndex < blocksArray.size(); nIndex++) {
            if(maxDistance == nAreaArray[nIndex]){
                break;
            }
        }

        //
        //3. 根据最大的block所在的位置来判断
        HuskyLens.Block block = blocksArray.get(nIndex);
        int nXPos = block.x;
        if(nXPos < LEFT_AREA_X){
            tagDirection = TagDirection.LEFT;
        } else if(LEFT_AREA_X <= nXPos && nXPos <= RIGHT_AREA_X){
            tagDirection = TagDirection.MIDDLE;
        } else if(nXPos > RIGHT_AREA_X){
            tagDirection = TagDirection.RIGHT;
        }

        return true;
    }

    private void calculateDirectionInternal(ArrayList<HuskyLens.Block> blocksArray){
//        if(BLOCK_NUM_DETECTED_1 == blocksArray.size()){
//            calObjectedDetectedBlocksDirection(blocksArray);
//            bDetectedtag = true;
//        } else if(BLOCK_NUM_DETECTED_3 == blocksArray.size()){
//            cal3DetectedBlocksDirection(blocksArray);
//            bDetectedtag = true;
//        } else if(BLOCK_NUM_DETECTED_4 == blocksArray.size()){
//            cal4DetectedBlocksDirection(blocksArray);
//            bDetectedtag = true;
//        } else {
//            telemetry.addData("Block ", "detected block nums ERROR!!!");
//        }

        if(BLOCK_NUM_DETECTED_1 <= blocksArray.size()){
            calBlocksDirection(blocksArray);
        } else {
            telemetry.addData("Block ", "detected block nums ERROR!!!");
        }
    }

    private void calculateDirection(HuskyLens.Block[] blocks){
        bDetectedtag = false;
        if(null == blocks || 0 == blocks.length){
            telemetry.addData(">>", "blocks detected ERROR!!!");
            return;
        }

        ArrayList<HuskyLens.Block> blocksArray = this.getValidBlocks(blocks);
        telemetry.addData("Block count", blocksArray.size());

        calculateDirectionInternal(blocksArray);

        for (int i = 0; i < blocksArray.size(); i++) {
            telemetry.addData("Block", blocksArray.get(i).toString());
        }

        if(bDetectedtag){
            telemetry.addData("Direction(0 unkown, 1 left, 2 middle, 3 right) ", tagDirection);
        } else {
            telemetry.addData(">>>","NO detected ERROR!!");
        }
        telemetry.update();
    }

    public TagDirection getTagDirection(){
        return tagDirection;
    }
}