package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private static PIDCoefficients coefficients;
    private double error, lastError;
    private double I, D, P;
    private ElapsedTime time;
    public PIDController(PIDCoefficients c){
        coefficients = c;
        time.reset();
        error = 0;
        lastError = 0;
    }

    public double calculate(double currentPos, double targetPos){
        error = targetPos - currentPos;

        I += error;

        D = (error - lastError) / time.seconds();

        lastError = error;
        time.reset();
        return coefficients.p * error + coefficients.i * I + coefficients.d * D;

    }
}
