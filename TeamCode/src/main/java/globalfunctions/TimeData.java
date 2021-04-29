package globalfunctions;

import java.sql.Time;
import java.util.ArrayList;

import androidx.annotation.NonNull;

//Used to define data that changes over time
public class TimeData {
    //Name of data
    public String name;
    //Time stamps of data
    public ArrayList<String> timeStamps;
    //Data
    public ArrayList<String> data;

    //Construct TimeData using arraylist of doubles
    public TimeData(String name, ArrayList<double[]> data, ArrayList<Double> timeStamps){
        ArrayList<String> out = new ArrayList<>();
        ArrayList<String> out2 = new ArrayList<>();

        for(double[] i: data){
            if(data.get(0).length == 3) {
                out.add("(" + i[0] + "," + i[1] + "," + i[2] + ")");
            }else if(data.get(0).length == 2){
                out.add("(" + i[0] + "," + i[1] +  ")");
            }else if(data.get(0).length == 1){
                out.add("(" + i[0] +  ")");
            }
        }
        for(double j: timeStamps){
            out2.add(Double.toString(j));
        }
        this.name = name;
        this.data = out;
        this.timeStamps = out2;
    }
    //Construct TimeData using arraylist of doubles without timestamps
    public TimeData(String name, ArrayList<double[]> data, boolean what){
        ArrayList<String> out = new ArrayList<>();
        ArrayList<String> out2 = new ArrayList<>();

        for(double[] i: data){
            if(data.get(0).length == 3) {
                out.add("(" + i[0] + "," + i[1] + "," + i[2] + ")");
            }else if(data.get(0).length == 2){
                out.add("(" + i[0] + "," + i[1] +  ")");
            }else if(data.get(0).length == 1){
                out.add("(" + i[0] +  ")");
            }
        }
        this.name = name;
        this.data = out;
        this.timeStamps = out2;
    }
    //Construct TimeData using name and data
    public TimeData(String name, ArrayList<String> data){
        this.name = name;
        this.data = data;
    }
    //Get all of the data zipped into one list of strings
    public ArrayList<String> getZipped(){
        ArrayList<String> out = new ArrayList<>();
        for(int i = 0; i < timeStamps.size(); i++){
            out.add(timeStamps.get(i) + " : " + data.get(i));
        }
        return out;
    }
}
