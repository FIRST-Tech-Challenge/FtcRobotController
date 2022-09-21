package org.firstinspires.ftc.teamcode.globals;

public enum Alliance {
    INSTANCE;

    public enum AllianceTeam {
        RED,
        BLUE
    }

    private AllianceTeam selectedAllianceTeam = AllianceTeam.RED;

    public void setAllicanceTeam(AllianceTeam allianceTeam){
        selectedAllianceTeam = allianceTeam;
    }

    public AllianceTeam getAllianceTeam(){
        return selectedAllianceTeam;
    }

    public static Alliance getInstance(){
        return INSTANCE;
    }
}
