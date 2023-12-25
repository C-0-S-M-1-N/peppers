package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class BetterMotor {
    public enum RUN_MODE{
        PID,
        NO_PID
    }
    private DcMotorEx motor;
    private DcMotorEx encoder;
    private boolean Reverse;
    private RUN_MODE MODE;

    public PIDFCoefficients pidfCoefficients;

    public BetterMotor(DcMotorEx Motor, DcMotorEx Encoder, boolean reverse, RUN_MODE m){
        Reverse = reverse;
        MODE = m;

        MotorConfigurationType mct = Motor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        Motor.setMotorType(mct);

        switch (MODE){
            case NO_PID:
                Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case PID:
                Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Motor.setTargetPosition(0);
                Motor.setPower(1);
                break;
        }

    }

    public void update(){

    }
}
