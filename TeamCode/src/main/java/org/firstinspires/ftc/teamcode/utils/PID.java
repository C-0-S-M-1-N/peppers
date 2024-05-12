package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double error, lastError, integralSum, derivative, out;
    private double encoderPosition, reference;

    public PID(double ki, double kp, double kd, DcMotor m) {

        ElapsedTime timer = new ElapsedTime();
        while(m.getCurrentPosition() != m.getTargetPosition()){
            encoderPosition = m.getTargetPosition();
            reference = m.getTargetPosition();
            error = reference - encoderPosition;
            derivative = (error - lastError) / timer.seconds();
            integralSum += error * timer.seconds();
            out = (kp*error) + (ki*integralSum) + (kd * derivative);
            m.setPower(out);
            lastError = error;
            timer.reset();
        }
    }
}
