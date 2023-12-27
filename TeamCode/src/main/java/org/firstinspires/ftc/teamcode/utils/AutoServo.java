package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Exceptions.OverTheLimitException;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

public class AutoServo {
    public enum type{
        GOBILDA,
        DS,
        MICRO_SERVO,
        AXON
    }
    SERVO_PORTS servo;
    private boolean revesed;
    private boolean isOnControlHub = true;
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
    public AutoServo(SERVO_PORTS port, boolean CHub, boolean rev, double initPos, type T){
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
            case MICRO_SERVO:
                MAX_ANGLE = 180;
                break;
            default:
                MAX_ANGLE = 0;
                break;
        }
        position = initPos;
        revesed = rev;
        isOnControlHub = CHub;
    }

    public void update(){
        position += step * deltaTime.seconds() * (targetPosition - position);

        if(isOnControlHub) {
            ControlHub.setServoPosition(servo, position);
            if(revesed) ControlHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ControlHub.setServoDirection(servo, Servo.Direction.FORWARD);
        }
        else {
            ExpansionHub.setServoPosition(servo, position);
            if(revesed) ExpansionHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ExpansionHub.setServoDirection(servo, Servo.Direction.FORWARD);
        }

        deltaTime.reset();
    }

    public void setAngle(double angle) {
        position = angle/MAX_ANGLE;
    }
    public double getAngle(){
        return position*MAX_ANGLE;
    }

}
