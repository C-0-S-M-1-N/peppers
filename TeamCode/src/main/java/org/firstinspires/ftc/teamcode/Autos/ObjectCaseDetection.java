package org.firstinspires.ftc.teamcode.Autos;

public class ObjectCaseDetection {
    public enum Team{
        RED,
        BLUE
    }

    public enum Case{
        LEFT,
        MIDDLE,
        RIGHT
    }
    public Team team;
    public ObjectCaseDetection(Team team){
        // TODO - implement it
        // camera funny

        this.team = team;
    }

    public void run(){

    }

    public Case getCase(){

        return Case.LEFT;
    }
}
