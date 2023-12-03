package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;

@Config
public class ElevatorArm implements Part {
    public enum STATES{
        TO_BACKDROP,
        TO_FEED,
        STATIC
    }
    public STATES STATE;
    public static boolean InWork = true;
    public static boolean ReverseVirtual1 = false, ReverseVirtual2 = false;
    private Telemetry telemetry;
    private Servo virtual1, virtual2;

    private static double   toBackdrop = 0,
                            toFeed = 0,
                            position = 0,
                            staticPos = 0,
                            inUsePosition;

    private void setRevesed(){
        if(ReverseVirtual1) virtual1.setDirection(Servo.Direction.REVERSE);
        else virtual1.setDirection(Servo.Direction.FORWARD);

        if(ReverseVirtual2) virtual2.setDirection(Servo.Direction.REVERSE);
        else virtual2.setDirection(Servo.Direction.FORWARD);
    }
    public ElevatorArm(HardwareMap hm, Telemetry tele){

        telemetry = tele;
        virtual1 = hm.get(Servo.class, "virtual1");
        virtual2 = hm.get(Servo.class, "virtual2");

        setRevesed();

        virtual1.setPosition(staticPos);
        virtual2.setPosition(staticPos);

        setToStatic();

    }
    @Override
    public void update(){
        setRevesed();

        switch (STATE){
            case STATIC:
                virtual1.setPosition(staticPos);
                virtual1.setPosition(staticPos);
                break;
            default:
                virtual1.setPosition(position);
                virtual2.setPosition(position);
                break;
        }
    }
    public void setToBackdrop(){
        inUsePosition = toBackdrop;
        STATE = STATES.TO_BACKDROP;
    }
    public void setToFeed(){
        inUsePosition = toFeed;
        STATE = STATES.TO_FEED;
    }
    public void setToStatic(){
        inUsePosition = staticPos;
        STATE = STATES.STATIC;
    }
    public void setPrecentageToEnd(double precent){
        position = precent * inUsePosition;
    }
    @Override
    public void update_values(){
        // nothing to do here :)
    }
}
