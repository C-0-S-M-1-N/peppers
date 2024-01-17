package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoSensor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.Objects;

@Config
public class Grippers implements Part {
    public boolean Disable = false;
    public boolean manual = false;
    public enum STATES{
        CLOSED,
        OPEN
    }
    public STATES STATE;

    public static double closeClaw = 0.35;
    private String ID = "null";

    private AutoServo claw;
    private DigitalChannel sensor;
    private Telemetry telemetry;

    public Grippers(AutoServo p, DigitalChannel sens, Telemetry tele){
        telemetry = tele;
        claw = p;
        STATE = STATES.OPEN;
        sensor = sens;
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public Grippers(AutoServo p, DigitalChannel sens, Telemetry tele, String id){
        ID = id;
        telemetry = tele;
        claw = p;
        STATE = STATES.OPEN;
        sensor = sens;
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void update(){
        if(Disable) return;
//        if(!manual){
//            if(sensor.getState()) {
//                if(STATE == STATES.CLOSED){
//                    if(Objects.equals(ID, "LEFT"))
//                        Controls.currentState = Controls.RumbleEffectPlay.LeftGot;
//                    else Controls.currentState = Controls.RumbleEffectPlay.RightGot;
//
//                }
//                STATE = STATES.OPEN;
//            }
//            else {
//                if(STATE == STATES.OPEN){
//                    if(Objects.equals(ID, "LEFT"))
//                        Controls.currentState = Controls.RumbleEffectPlay.LeftLost;
//                    else Controls.currentState = Controls.RumbleEffectPlay.RightLost;
//                }
//                STATE = STATES.CLOSED;
//            }
//            switch (STATE){
//                case OPEN:
//                    claw.setAngle(0);
//                    break;
//                case CLOSED:
//                    claw.setAngle(closeClaw);
//            }
//        }
        if(STATE == STATES.OPEN && !manual && !sensor.getState()){
            claw.setPosition(closeClaw);
            STATE = STATES.CLOSED;
        }
        claw.update();

    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        telemetry.addLine(ID + ":" +
                "\n\tsensor state: " + sensor.getState() +
                "\n\tgripper state: " + STATE.toString());
    }
    public void drop(){

        if(STATE == STATES.OPEN && manual){
            claw.setPosition(closeClaw);
            STATE = STATES.CLOSED;
        } else {
            claw.setAngle(0);
            STATE = STATES.OPEN;
        }
    }


}
