package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

public class AutoServo {
    public enum TYPE{
        DS,
        GOBILDA,
        MICRO_LEGO,
        AXON,
        UNKNOWN
    }
    private Hubs hub;
    private final SERVO_PORTS port;
    private final boolean isReversed;
    private double MAX_ANGLE;
    private double position;
    private final double initialPosition;

    public AutoServo(SERVO_PORTS port, double initialPosition, boolean isReversed, Hubs hub, TYPE Type){
        this.port = port;
        this.initialPosition = initialPosition;
        this.isReversed = isReversed;
        this.hub = hub;

        switch (Type){
            case DS:
                MAX_ANGLE = 270;
                break;
            case GOBILDA:
                MAX_ANGLE = 300;
                break;
            case MICRO_LEGO:
                MAX_ANGLE = 180;
                break;
            case AXON:
                MAX_ANGLE = 355;
                break;
            case UNKNOWN:
                MAX_ANGLE = 360;
                break;
        }
    }

    public void update(){
        switch (hub){
            case CONTROL_HUB:
                ControlHub.setServoPosition(port, position);
                ControlHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                break;
            case EXPANSION_HUB:
                ExpansionHub.setServoPosition(port, position);
                ExpansionHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                break;
        }
    }

    public void setAngle(double angle){ position = angle / MAX_ANGLE + initialPosition; }
    public void setPosition(double pos){ position = pos + initialPosition; }

    public double getAngle(){ return (position - initialPosition) * MAX_ANGLE; }
    public double getPosition(){ return position - initialPosition; }

}
