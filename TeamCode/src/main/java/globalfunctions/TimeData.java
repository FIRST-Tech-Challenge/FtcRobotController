package globalfunctions;

import java.sql.Time;
import java.util.ArrayList;

public class TimeData {
    public String name;
    public String user = "me";
    public ArrayList<String> timeStamps;
    public ArrayList<String> data;


    public TimeData(String name, ArrayList<double[]> poses, ArrayList<Double> timeStamps){
        ArrayList<String> out = new ArrayList<>();
        ArrayList<String> out2 = new ArrayList<>();

        for(double[] i: poses){
            out.add("(" +Double.toString(i[0]) + "," +  Double.toString(i[1]) + "," + Double.toString(i[2] ) + ")");
        }
        for(double j: timeStamps){
            out2.add(Double.toString(j));
        }
        this.name = name;
        this.data = out;
        this.timeStamps = out2;
    }

    public TimeData(String name, ArrayList<String> data){
        this.name = name;
        this.data = data;
    }

    public ArrayList<String> getZipped(){
        ArrayList<String> out = new ArrayList<>();
        for(int i = 0; i < timeStamps.size(); i++){
            out.add(timeStamps.get(i) + " : " + data.get(i));
        }
        return out;
    }
}
