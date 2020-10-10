package org.firstinspires.ftc.teamcode.utility;

public class maximum {
    public double[] nums;

    public maximum(double... nums){
        this.nums = nums;
    }

    public double max(){
        if(nums.length == 0) return Double.NaN;

        double max = nums[0];
        for(double n:nums){
            max = Math.max(n, max);
        }
        return max;
    }

    public void squishIntoRange(double range){
        double max = max();
        if(max < range) return;

        for(int i = 0; i<nums.length; i++){
            nums[i] /= max;
        }
    }
}
