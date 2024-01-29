package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private PIDCoefficients pidCoefficients;
    private double targetPosition = 0;
    private double error, lastError, maxActuatorOutput, Isum = 0;
    private ElapsedTime et;
    private double threshold = 2;
    private boolean clampI = false;

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
    public void setThreshold(double th){
        threshold = th;
    }

    public double calculatePower(double currentPosition){
        error = targetPosition - currentPosition;
        double time = et.seconds();

        double P = error;
        double D = (error - lastError) / time;
        Isum += P * time;
        double r = pidCoefficients.p * P + pidCoefficients.i * Isum + pidCoefficients.d * D;

        if(isOverSaturated(r) && error * r > 0){
            Isum -= P * time;
            clampI = true;
        } else clampI = false;

//        if(abs(error) <= threshold) {
//            Isum = 0;
//        } // sussy code - codrin

        et.reset();

        lastError = error;
        return  pidCoefficients.p * P
                + pidCoefficients.i * Isum * (clampI ? 0 : 1)
                + pidCoefficients.d * D;
    }
    private boolean isOverSaturated(double output){
        return output > maxActuatorOutput;
    }
    public void setTargetPosition(double pos){
        targetPosition = pos;
        Isum = 0;
    }
    public void setMaxActuatorOutput(double mao){
        maxActuatorOutput = mao;
    }
}
