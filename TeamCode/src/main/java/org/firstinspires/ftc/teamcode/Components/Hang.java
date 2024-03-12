package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

// FORWARD -retract

@Config
public class Hang {
    public static double maxUp = 6350;
    public Hang(){
        ControlHub.setEncoderDirection(ENCODER_PORTS.E3, Encoder.Direction.REVERSE);
        ExpansionHub.setServoPosition(SERVO_PORTS.S4, 185/270.f);
        ExpansionHub.setServoPosition(SERVO_PORTS.S3, 190/270.f);
    }

    public void update(){
        ControlHub.setMotorPower(MOTOR_PORTS.M3, Controls.HangLevel);
        if(Controls.Hang){
            ExpansionHub.setServoPosition(SERVO_PORTS.S4, 30/270.f);
            ExpansionHub.setServoPosition(SERVO_PORTS.S3, 100/270.f);
        }
    }
}
