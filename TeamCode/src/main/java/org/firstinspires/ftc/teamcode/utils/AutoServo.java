package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Exceptions.OverTheLimitException;

public class AutoServo {
    public enum type{
        GOBILDA,
        DS,
        AXON
    }
    private static Servo servo;
    private static boolean revesed;
    private double position, targetPosition;
    private static double step = 0.001;
    private ElapsedTime deltaTime;
    public static int MAX_ANGLE;
    /*
    * 270 deg -> virtual servos
    * 300 deg -> gobilda servos
    * 355 deg -> axon servos
    * */

    public void setPosition(double p){
        position = p;
    }
    public double getPosition(){
       return position;
    }
    public AutoServo(Servo s, boolean rev, double initPos, type T){
        switch(T){
            case GOBILDA:
                MAX_ANGLE = 300;
                break;
            case DS:
                MAX_ANGLE = 270;
                break;
            case AXON:
                MAX_ANGLE = 355;
                break;
            default:
                MAX_ANGLE = 0;
                break;
        }
        position = initPos;
        revesed = rev;
        servo = s;
    }

    public void update(){
        position += step * deltaTime.seconds() * (targetPosition - position);

        servo.setPosition(position);

        if(revesed) servo.setDirection(Servo.Direction.REVERSE);
        else servo.setDirection(Servo.Direction.FORWARD);

        deltaTime.reset();
    }

    public void setAngle(double angle) {
        position = angle/MAX_ANGLE;
    }
    public double getAngle(){
        return position*MAX_ANGLE;
    }

}
