package org.darbots.darbotsftclib.libcore.integratedfunctions.logger;


import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.json.simple.parser.JSONParser;
import org.json.simple.JSONArray;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.util.Collection;
import java.util.Map;

public class RobotLogFile {
    private File m_File;
    private JSONArray m_OpModeRunLogs;
    public RobotLogFile(File file){
        this.m_File = file;
        __readLogFile();
    }

    public RobotLogFile(String filename){
        this.m_File = FTCFileIO.getSettingFile(filename);
        __readLogFile();
    }

    protected void __readLogFile(){
        File logFile = this.m_File;
        String logFileContent = FTCFileIO.readFile(logFile);
        if(logFileContent == null || logFileContent.isEmpty()){
            m_OpModeRunLogs = new JSONArray();
        }else {
            JSONParser Parser = new JSONParser();
            try {
                Object ParsedContent = Parser.parse(logFileContent);
                if(ParsedContent instanceof JSONArray) {
                    JSONArray parsedContent = (JSONArray) ParsedContent;
                    m_OpModeRunLogs = new JSONArray();
                    for(Object i : parsedContent){
                        if(i instanceof Map){
                            m_OpModeRunLogs.add(new OpModeRunLog((Map) i));
                        }
                    }
                }else{
                    m_OpModeRunLogs = new JSONArray();
                }
            } catch (ParseException e) {
                m_OpModeRunLogs = new JSONArray();
            } catch (Exception e) {
                m_OpModeRunLogs = new JSONArray();
            }
        }
    }

    public Collection<OpModeRunLog> getAllLogs(){
        return this.m_OpModeRunLogs;
    }

    public OpModeRunLog addNewRunLog(){
        OpModeRunLog newRunLog = null;
        if(GlobalRegister.runningOpMode != null) {
            newRunLog = new OpModeRunLog(GlobalRegister.runningOpMode.getClass());
        }else{
            newRunLog = new OpModeRunLog(DarbotsBasicOpMode.class);
        }
        this.m_OpModeRunLogs.add(newRunLog);
        return newRunLog;
    }

    public int numberOfOpModeLogs(){
        return this.m_OpModeRunLogs.size();
    }

    public OpModeRunLog getRunLog(int Index){
        return (OpModeRunLog) this.m_OpModeRunLogs.get(Index);
    }

    public OpModeRunLog removeRunLog(int Index){
        return (OpModeRunLog) (this.m_OpModeRunLogs.remove(Index));
    }

    public OpModeRunLog addRunLog(OpModeRunLog addedLog){
        this.m_OpModeRunLogs.add(addedLog);
        return addedLog;
    }

    public void saveToFile(){
        File logFile = this.m_File;
        String logFileContent = this.m_OpModeRunLogs.toJSONString();
        FTCFileIO.writeFile(logFile,logFileContent);
    }
}
