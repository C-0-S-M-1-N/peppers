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
        AXON
    }
    SERVO_PORTS servo;
    private boolean revesed;
    private boolean isOnControlHub = true;
    private double position, targetPosition;
    public static double step = 0.1;
    private ElapsedTime deltaTime;
    public static int MAX_ANGLE;
    /*
    * 270 deg -> virtual servos
    * 300 deg -> gobilda servos
    * 355 deg -> axon servos
    * */

    public void setPosition(double p){
        targetPosition = p;
    }
    public double getPosition(){
       return position;
    }
    public AutoServo(SERVO_PORTS port, boolean CHub, boolean rev, double initPos, type T){
        deltaTime = new ElapsedTime();
        deltaTime.startTime();
        switch(T){
            case GOBILDA:
                MAX_ANGLE = 300;
                break;
            case DS:
                MAX_ANGLE = 270;
                step = 10;
                break;
            case AXON:
                MAX_ANGLE = 355;
                break;
            case MICRO_SERVO:
                MAX_ANGLE = 180;
                step = 0.007;
                break;
            default:
                MAX_ANGLE = 0;
                break;
        }
        position = initPos;
        revesed = rev;
        isOnControlHub = CHub;
        servo = port;
    }

    public void update(){

        double time = deltaTime.seconds();
        if((position <= targetPosition && targetPosition <= step * deltaTime.seconds() * signum(targetPosition - position)) ||
            (position >= targetPosition && targetPosition >= step * deltaTime.seconds() * signum(targetPosition - position)))
        {
            position = targetPosition;
        } else {
            position += step * deltaTime.seconds() * signum(targetPosition - position);
        }

        if(isOnControlHub) {
            if(revesed) ControlHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ControlHub.setServoDirection(servo, Servo.Direction.FORWARD);
            ControlHub.setServoPosition(servo, position);
        }
        else {
            if(revesed) ExpansionHub.setServoDirection(servo, Servo.Direction.REVERSE);
            else ExpansionHub.setServoDirection(servo, Servo.Direction.FORWARD);
            ExpansionHub.setServoPosition(servo, position);
        }

        deltaTime.reset();
    }

    public void setAngle(double angle) {
        targetPosition = angle/MAX_ANGLE;
    }
    public double getAngle(){
        return position*MAX_ANGLE;
    }

}
