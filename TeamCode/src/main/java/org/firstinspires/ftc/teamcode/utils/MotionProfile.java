package org.firstinspires.ftc.teamcode.utils;

public class MotionProfile {
    public enum STATES{
        ACCELERATING,
        DECCELERATING,
        STATIC,
        EOM;
    }
    STATES STATE;
    private double position, velocity, acceleration, finalPos, usedAcceleration;
    private double maxVelocity;

    public MotionProfile(double MaxVelocity, double Acceleration){
        maxVelocity = MaxVelocity;
        acceleration = Acceleration;
    }
    public void startMotion(double FinalPos){
        finalPos = FinalPos;
        position = 0;
        velocity = 0;
        STATE = STATES.ACCELERATING;
        usedAcceleration = acceleration;
    }
    public void update(){
        switch (STATE){
            case ACCELERATING:
                if(velocity >= maxVelocity){
                    maxVelocity = velocity;
                    STATE = STATES.STATIC;
                    usedAcceleration = 0;
                }
                break;
            case STATIC:
                if(position >= maxVelocity*maxVelocity/(2 * acceleration)){
                    usedAcceleration = -acceleration;
                    STATE = STATES.DECCELERATING;
                }
                break;
            case DECCELERATING:
                if(motionDone()){
                    STATE = STATES.EOM;
                }
                if(velocity < 0){
                    velocity = 0;
                }
                break;
            case EOM:
                return;
        }
        position += velocity;
        velocity += usedAcceleration;
    }

    public double getPositon(){ return position; }
    public boolean motionDone(){
        return position >= finalPos;
    }
}
