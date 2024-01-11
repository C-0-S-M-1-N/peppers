package org.firstinspires.ftc.teamcode.Parts;

public class MotionProfile {
    private double maxVelocity, acceleration;
    private double currentPosition, targetPosition,
        accelerationTime, constantTime, deccelarationTime, distance,
        maxVeloPosition, deccelerationPosition;
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

    }
}
