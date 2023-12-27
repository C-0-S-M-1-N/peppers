package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoSensor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

public class Grippers implements Part {
    public boolean Disable = false;
    public enum STATES{
        CLOSED,
        OPEN
    }
    public STATES STATE;

    private AutoSensor sensor;
    private AutoServo claw;
    private double closeClaw = 90;
    private int ticks_closed = 0, MT = 3;

    public Grippers(AutoServo s, AutoSensor a){
        sensor = a;
        claw = s;
        STATE = STATES.OPEN;
    }

    @Override
    public void update(){
        if(Disable) return;
        sensor.update();

        switch (STATE){
            case OPEN:
                if(sensor.ObjDetected()){
                    STATE = STATES.CLOSED;
                }
                claw.setAngle(0);
            case CLOSED:
                if(!sensor.ObjDetected() && ticks_closed < MT){
                    STATE = STATES.OPEN;
                }
                claw.setAngle(closeClaw);
        }

    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){

    }


}
