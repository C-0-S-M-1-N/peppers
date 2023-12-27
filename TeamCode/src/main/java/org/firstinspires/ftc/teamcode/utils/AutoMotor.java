package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class AutoMotor {
    public DcMotorEx Motor;
    public boolean isReversed;
    private double power;
    private void setDirection(){
        Motor.setDirection(isReversed ?
                DcMotorSimple.Direction.REVERSE :
                DcMotorSimple.Direction.FORWARD);
    }
    public AutoMotor(DcMotorEx motorFromMap, boolean reversed){
        isReversed = reversed;
        Motor = motorFromMap;

        MotorConfigurationType mct = Motor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        Motor.setMotorType(mct);

        setDirection();
    }

    public void update(){
        setDirection();
        Motor.setPower(power);
    }

    public void setPower(double p){
        power = p;
    }
}
