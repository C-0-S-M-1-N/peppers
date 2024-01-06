package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Exceptions.OverTheLimitException;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

@Config
public class AutoServo {
    public enum type{
        GOBILDA,
        DS,
        MICRO_SERVO,
        AXON,
        GOBILDA_SPEED
    }
    type Type;
    SERVO_PORTS servo;
    private boolean revesed;
    private boolean isOnControlHub = true;
    private double position, targetPosition;
    public double step = 1;
    public static int MAX_ANGLE;
    private MotionProfile profile;
    public ElapsedTime time;
    /*
    * 270 deg -> virtual servos
    * 300 deg -> gobilda servos
    * 355 deg -> axon servos
    * */

    public void setPosition(double p){
        targetPosition = p;
        if(position != targetPosition)
            profile.setMotion(position, targetPosition, 0);
    }
    public double getPosition(){
       return position;
    }
    public AutoServo(SERVO_PORTS port, boolean CHub, boolean rev, double initPos, type T){
        profile = new MotionProfile(10, 7, 7);
        time = new ElapsedTime();
        switch(T){
            case GOBILDA:
                step = 8;
                MAX_ANGLE = 300;
                break;
            case GOBILDA_SPEED:
                step = 10;
                MAX_ANGLE = 300;
                break;
            case DS:
                MAX_ANGLE = 270;
                step = 5;
//                step = 0.5;
                break;
            case AXON:
                MAX_ANGLE = 355;
                break;
            case MICRO_SERVO:
                MAX_ANGLE = 180;
                step = 100;
                break;
            default:
                MAX_ANGLE = 0;
                break;
        }
        position = initPos;
        revesed = rev;
        isOnControlHub = CHub;
        servo = port;
        if(isOnControlHub)
            if(revesed) ControlHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ControlHub.setServoDirection(servo, Servo.Direction.FORWARD);
        else
            if(revesed) ExpansionHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ExpansionHub.setServoDirection(servo, Servo.Direction.FORWARD);

        Type = T;
    }

    public void update(){

        if(Type == type.MICRO_SERVO) position = targetPosition;
//        else position += (targetPosition - position) * step * time.seconds();
//        if(Type == type.DS){
//            if(targetPosition - 0.1 <= position && position <= targetPosition + 0.1)
//                position = targetPosition;
//        }
        position = profile.getPosition();
        if(Type == type.MICRO_SERVO){
            position = targetPosition;
        }

//        if(targetPosition - 0.01 <= position && position <= targetPosition + 0.01) position = targetPosition;

        if(isOnControlHub) {
            ControlHub.setServoPosition(servo, position);
        }
        else {
            ExpansionHub.setServoPosition(servo, position);
        }
        time.reset();
        profile.update();
    }

    public void setAngle(double angle) {
        targetPosition = angle/MAX_ANGLE;
        if(position != targetPosition)
            profile.setMotion(position, targetPosition, 0);
    }
    public double getAngle(){
        return position*MAX_ANGLE;
    }
    public double getTargetPosition(){
        return targetPosition;
    }
    public double getTargetAngle(){
        return targetPosition * MAX_ANGLE;
    }

}
