package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.opencv.core.Mat;

@Config
public class ElevatorArm implements Part {
    public enum STATES{
        IDLE,
        DOWN,
        UP
    }
    public STATES STATE;
    public static boolean InWork = true;
    public static boolean ReverseVirtual1 = false, ReverseVirtual2 = true;
    private Telemetry telemetry;
    private Servo virtual1, virtual2;
    private int sign = 0;

    private static double   toBackdrop = 0.95,
                            toFeed = 0,
                            targetPosition = 0,
                            staticPos = 0.23,
                            currentPosition;

    private void setReversed(){
        if(ReverseVirtual1) virtual1.setDirection(Servo.Direction.REVERSE);
        else virtual1.setDirection(Servo.Direction.FORWARD);

        if(ReverseVirtual2) virtual2.setDirection(Servo.Direction.REVERSE);
        else virtual2.setDirection(Servo.Direction.FORWARD);
    }
    public ElevatorArm(HardwareMap hm, Telemetry tele){

        telemetry = tele;
        virtual1 = hm.get(Servo.class, "virtual1");
        virtual2 = hm.get(Servo.class, "virtual2");

        setReversed();

        virtual1.setPosition(staticPos);
        virtual2.setPosition(staticPos);

        setToStatic();

    }
    @Override
    public void update(){
        setReversed();

        switch (STATE){
            case DOWN:
                sign = -1;
                break;
            case UP:
                sign = 1;
                break;
            case IDLE:
                sign = 0;
                break;
        }

        virtual1.setPosition(currentPosition);
        virtual2.setPosition(currentPosition);
    }
    public void setToBackdrop(){
        targetPosition = toBackdrop;
        if(currentPosition > targetPosition) STATE = STATES.DOWN;
        else if(currentPosition < targetPosition) STATE = STATES.UP;
        else STATE = STATES.IDLE;
    }
    public void setToFeed(){
        targetPosition = toFeed;
        if(currentPosition > targetPosition) STATE = STATES.DOWN;
        else if(currentPosition < targetPosition) STATE = STATES.UP;
        else STATE = STATES.IDLE;
    }
    public void setToStatic(){
        targetPosition = staticPos;
        if(currentPosition > targetPosition) STATE = STATES.DOWN;
        else if(currentPosition < targetPosition) STATE = STATES.UP;
        else STATE = STATES.IDLE;
    }
    public void setPrecentageToEnd(double precent){
        currentPosition += ((currentPosition - targetPosition) * precent);
    }
    @Override
    public void update_values(){
        // nothing to do here :)
    }
}
