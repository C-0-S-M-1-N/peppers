package org.firstinspires.ftc.teamcode.Parts;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

/*
* MAP:
* 0 - BR
* 1 - BL
* 2 - FR
* 3 - FL
*
* */
@Config
public class MecanumDrive{
    public static boolean Disable = false;
    public static double ACC = 0.1;
    private Telemetry telemetry;
    public MecanumDrive(Telemetry tele){
        telemetry = tele;
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M0, DcMotorSimple.Direction.REVERSE);
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
    }

    public void update(double left_stick_y, double left_stick_x,
                       double right_trigger, double left_trigger, boolean boost){
        if(Disable) return;

        double rotation = right_trigger - left_trigger;
        double denominator = Math.max(abs(left_stick_x) + abs(left_stick_y) + abs(rotation), 1);

        double m0Power = (-left_stick_x - left_stick_y + rotation)/denominator;
        double m2Power = (-left_stick_x + left_stick_y - rotation)/denominator;
        double m1Power = (-left_stick_x + left_stick_y + rotation)/denominator;
        double m3Power = (-left_stick_x - left_stick_y - rotation)/denominator;

        if(boost){
            ACC = 1;
        } else ACC = 0.6;

        ExpansionHub.setMotorPower(MOTOR_PORTS.M0, m0Power * ACC);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M2, m2Power * ACC);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M1, m1Power * ACC);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M3, m3Power * ACC);

    }



}
