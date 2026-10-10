package org.firstinspires.ftc.teamcode.drivesupport;

public class Robot_Placement {
    public String target_tag;
    public void init(int position_id){
        if (position_id == 1){
            // indicates red-1
            target_tag = "RED SCORING";
            int[] tag_id = {30,31,32,33,34,35,36,37};
        }
        if (position_id == 2){
            // indicates red-2
            target_tag = "RED SCORING";
            int[] tag_id = {30,31,32,33,34,35,36,37};
        }
        if (position_id == 3){
            // indicates blue-1
            target_tag = "BLUE SCORING";
            int[] tag_id = {40,41,42,43,44,45,46,47};
        }
        if (position_id == 4){
            // indicates blue-2
            target_tag = "BLUE SCORING";
            int[] tag_id = {40,41,42,43,44,45,46,47};
        }


    }
}
