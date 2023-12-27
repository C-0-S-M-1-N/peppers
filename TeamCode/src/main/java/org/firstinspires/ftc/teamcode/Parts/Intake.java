package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

import java.util.ResourceBundle;

/*
* MAP:
* 3 - INTAKE
* */
@Config
public class Intake implements Part {
    public static boolean Disabled = false;
    public enum STATES{
        IDLE,
        REVERSE,
        FORWARD
    }
    public STATES STATE;
    public static double maxTrashHold = 0.5;
    private double usedCurrent = 0;
    public Intake(){
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
                ControlHub.setMotorPower(MOTOR_PORTS.M3, 0);
                break;
            case FORWARD:
                ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                break;
            case REVERSE:
                ControlHub.setMotorPower(MOTOR_PORTS.M3, -1);
                break;
        }

    }
    @Override
    public void update_values(){
        usedCurrent = ControlHub.getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.MILLIAMPS);
    }
    @Override
    public void runTelemetry(){}
}
