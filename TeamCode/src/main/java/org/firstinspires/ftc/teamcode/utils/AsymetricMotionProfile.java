package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AsymetricMotionProfile {
    private double T0, T1, T2;
    private ElapsedTime time;
    public double Acceleration, Decceleration, MaxVelocity;
    private double MaxUsedVelocity, initialPosition;
    public double targetPosition;

    public AsymetricMotionProfile(double A, double D, double MV){
        Acceleration = A;
        Decceleration = D;
        MaxVelocity = MV;
    }
    public void startMotion(double p0, double pf){
        double distance = Math.abs(pf - p0);
        if(distance == 0) return;
        initialPosition = p0;
        targetPosition = pf;

        MaxUsedVelocity = Math.min(MaxVelocity, Math.sqrt((2 * distance * Acceleration * Decceleration) / (Acceleration + Decceleration)));
        T0 = MaxUsedVelocity / Acceleration;
        T2 = MaxUsedVelocity / Decceleration;
        T1 = Math.max(distance / MaxUsedVelocity - (T0 + T2) / 2, 0);
        time.reset();
    }
    private double p(double t){
        if(t <= T0) return Acceleration / 2 * t * t;
        if(t <= T0 + T1) return Acceleration / 2 * T0 * T0 + MaxUsedVelocity * (t - T0);
        if(t <= T0 + T1 + T2) return Acceleration / 2 * T0 * T0 + MaxUsedVelocity * (t - T0) - Decceleration / 2 * (t - T0 - T1) * (t - T0 - T1);
        return p(T0 + T1 + T2);
    }
    public double getPosition(){
        return initialPosition + Math.signum(targetPosition - initialPosition) * p(time.seconds());
    }
    public double getRemaningTime(){
        return Math.max(time.seconds() - T0 - T1 - T2, 0);
    }
    public boolean motionEnded(){
        return getRemaningTime() == 0;
    }
}
