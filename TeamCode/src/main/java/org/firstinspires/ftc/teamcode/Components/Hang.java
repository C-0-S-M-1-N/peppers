package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

// FORWARD -retract

@Config
public class Hang {
    public static double maxUp = 6350;
    public Hang(){
        ControlHub.setEncoderDirection(ENCODER_PORTS.E3, Encoder.Direction.REVERSE);
    }

    public void update(){
        ControlHub.setMotorPower(MOTOR_PORTS.M3, Controls.HangLevel);
    }
}
