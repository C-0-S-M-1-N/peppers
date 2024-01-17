package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private PIDCoefficients pidCoefficients;
    private double targetPosition = 0;
    private double error, lastError, maxActuatorOutput, Isum = 0;
    private ElapsedTime et;

    public PIDController(PIDCoefficients pidcoef){
        pidCoefficients = pidcoef;
        error = 0;
        lastError = 0;
        maxActuatorOutput = 1; // default for FTC motors/servos
        et = new ElapsedTime();
    }
    public PIDController(double p, double i, double d){
        this(new PIDCoefficients(p, i, d));
    }
    public void setPidCoefficients(PIDCoefficients coeff){
        pidCoefficients = coeff;
    }

    public double calculatePower(double currentPosition){
        error = targetPosition - currentPosition;
        double time = et.seconds();

        double P = error;
        double D = (error - lastError) / et.seconds();
        Isum += P * time;
        double r = pidCoefficients.p * P + pidCoefficients.i * Isum + pidCoefficients.d * D;

        double ret = Math.max(r, maxActuatorOutput);
        if(ret != r && error * r > 0){ // Integral Clamping for ant-windup
            Isum -= P * time;
        }


        et.reset();

        lastError = error;
        return pidCoefficients.p * P + pidCoefficients.i * Isum + pidCoefficients.d * D;
    }
    public void setTargetPosition(double pos){
        targetPosition = pos;
        Isum = 0;
    }
    public void setMaxActuatorOutput(double mao){
        maxActuatorOutput = mao;
    }

    public double getError(){ return error; }
}
