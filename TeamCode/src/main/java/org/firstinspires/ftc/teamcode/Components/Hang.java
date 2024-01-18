package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

// FORWARD -retract

@Config
public class Hang {
    public Hang(){

    }

    public void update(){
        ControlHub.setMotorPower(MOTOR_PORTS.M3, Controls.HangLevel);
    }
}
