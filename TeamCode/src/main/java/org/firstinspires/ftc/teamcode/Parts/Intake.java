package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Part;

import java.util.ResourceBundle;

@Config
public class Intake implements Part {
    public static boolean Disabled = false;
    public enum STATES{
        IDLE,
        REVERSE,
        FORWARD
    }
    public STATES STATE;
    private DcMotorEx motor;
    public static double maxTrashHold = 0.5;
    private double usedCurrent = 0;
    public Intake(HardwareMap hm, Controls c){
        motor = hm.get(DcMotorEx.class, "intake_motor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorConfigurationType mct = motor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(mct);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        STATE = STATES.IDLE;
    }
    @Override
    public void update(){
        if(Disabled) return;
        if(usedCurrent > maxTrashHold){
            STATE = STATES.REVERSE;
        }

        switch (STATE){
            case IDLE:
                motor.setPower(0);
                break;
            case FORWARD:
                motor.setPower(1);
                break;
            case REVERSE:
                motor.setPower(-1);
        }

    }
    @Override
    public void update_values(){
        usedCurrent = motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
    @Override
    public void runTelemetry(){}
}
