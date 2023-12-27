package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDController {
    public static PIDCoefficients coefficients;
    private double error, lastError, treshHold;
    private double I, D, P;
    private final ElapsedTime time;
    public PIDController(PIDCoefficients c, double tH){
        treshHold = tH;
        time = new ElapsedTime();
        coefficients = c;
        time.reset();
        error = 0;
        lastError = 0;
    }

    public double calculate(double currentPos, double targetPos){
        error = targetPos - currentPos;
        if(Math.abs(error) <= treshHold) error = 0;

        P = coefficients.p * time.seconds();
        I += coefficients.i * error * time.seconds();
        D = coefficients.d * (error - lastError) / time.seconds();

        lastError = error;
        time.reset();
        return P + I + D;

    }
}
