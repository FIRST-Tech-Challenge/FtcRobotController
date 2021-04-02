package globalfunctions;

import java.util.ArrayList;

public class TimeData {
    public String name;
    public String user = "me";
    public ArrayList<String> timeStamps;
    public ArrayList<String> data;


    public TimeData(String name, ArrayList<String> data, ArrayList<String> timeStamps){
        this.name = name;
        this.data = data;
        this.timeStamps = timeStamps;
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
