package org.darbots.darbotsftclib.libcore.integratedfunctions;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;

public class FTCMemory {
    public static final File MEMORY_FILE = FTCFileIO.getFirstFolderFile("DarbotsOpModes.mem");
    public static JSONObject memoryMap = null;
    public static void initializeMemoryMap(){
        if(memoryMap != null){
            return;
        }
        File logFile = MEMORY_FILE;
        String logFileContent = FTCFileIO.readFile(logFile);
        if(logFileContent == null || logFileContent.isEmpty()){
            memoryMap = new JSONObject();
        }else {
            JSONParser Parser = new JSONParser();
            try {
                Object ParsedContent = Parser.parse(logFileContent);
                if(ParsedContent instanceof JSONObject) {
                    JSONObject parsedContent = (JSONObject) ParsedContent;
                    memoryMap = parsedContent;
                }else{
                    memoryMap = new JSONObject();
                }
            } catch (ParseException e) {
                memoryMap = new JSONObject();
            } catch (Exception e) {
                memoryMap = new JSONObject();
            }
        }
    }
    public static void saveMemoryMapToFile(){
        if(memoryMap == null){
            return;
        }
        String memoryString = memoryMap.toJSONString();
        FTCFileIO.writeFile(MEMORY_FILE,memoryString);
    }
    public static <T> T getSetting(Object key, T defaultValue){
        if(memoryMap == null){
            initializeMemoryMap();
        }
        Object objRetrieved = memoryMap.get(key);
        if(objRetrieved == null){
            return defaultValue;
        }else{
            try {
                T result = (T) objRetrieved;
                return result;
            }catch(ClassCastException e){
                return defaultValue;
            }
        }
    }
    public static <T> void setSetting(Object key, T value){
        if(memoryMap == null){
            initializeMemoryMap();
        }
        memoryMap.put(key,value);
    }
}
