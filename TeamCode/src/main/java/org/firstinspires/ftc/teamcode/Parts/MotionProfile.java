package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {
    private double maxVelocity, acceleration;
    private ElapsedTime time = new ElapsedTime();
    private double currentPosition, targetPosition,
        accelerationTime, constantTime, deccelarationTime, distance,
        maxVeloPosition, deccelerationPosition, velocity;
    public MotionProfile(double maxVelocity, double acceleration){
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
    }
    public void startMotion(double initialPos, double targetPos){
        accelerationTime = maxVelocity/acceleration;

        deccelarationTime = maxVelocity/acceleration;

        distance = Math.abs(initialPos - targetPos);

        if(distance < (accelerationTime + deccelarationTime) * maxVelocity / 2){

        } else {
            constantTime = (distance - (maxVelocity * maxVelocity) / acceleration) / maxVelocity;
        }
        maxVeloPosition = acceleration * accelerationTime;




        time.reset();
    }
    public void update(){
        if(time.seconds() < accelerationTime){
            velocity += acceleration;
        } else if(time.seconds() < constantTime){
            velocity += 0;
        } else if(time.seconds() < deccelarationTime){
            velocity -= acceleration;
        }
        currentPosition += velocity;
    }
}
