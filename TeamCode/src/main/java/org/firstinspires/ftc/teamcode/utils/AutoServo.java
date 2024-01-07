package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.scalb;
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
    public static boolean CacheOn = true;
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
    private boolean isOnControlHub = true, cached = false;
    private double position, targetPosition;
    public double step = 1;
    public static int MAX_ANGLE;

    public AutoServo(SERVO_PORTS port, boolean CHub, boolean rev, double initPos, type T){
        switch(T){
            case GOBILDA:
                MAX_ANGLE = 300;
                break;
            case DS:
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

        if(isOnControlHub) {
            if (revesed) ControlHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ControlHub.setServoDirection(servo, Servo.Direction.FORWARD);
        }
        else {
            if (revesed) ExpansionHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ExpansionHub.setServoDirection(servo, Servo.Direction.FORWARD);
        }

        update();

    }

    public void update(){
        cached = position == targetPosition && CacheOn;
        if(cached) return;

        if(isOnControlHub) {
            ControlHub.setServoPosition(servo, position);
        }
        else {
            ExpansionHub.setServoPosition(servo, position);
        }

    }
    public void setPosition(double p){
        targetPosition = p;
    }
    public double getPosition(){
        return position;
    }
    public void setAngle(double angle) {
        targetPosition = angle / MAX_ANGLE;
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
