package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

/*
* MAP:
* 0 - FL
* 1 - FR
* 2 - BL
* 3 - BR
*
* */
public class MecanumDrive{
    public static boolean Disable = false;
    private Telemetry telemetry;
    public MecanumDrive(Telemetry tele, HardwareMap hm){
        telemetry = tele;
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M0, DcMotorSimple.Direction.REVERSE);
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);

    }

    public void update(double left_stick_y, double left_stick_x,
                       double right_trigger, double left_trigger){
        if(Disable) return;
        double rotation = right_trigger - left_trigger;
        double denominator = Math.max(Math.abs(left_stick_x) + Math.abs(left_stick_y) + Math.abs(rotation), 1);


        ExpansionHub.setMotorPower(MOTOR_PORTS.M0, (left_stick_x + left_stick_y + rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M2, (left_stick_x - left_stick_y - rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M1, (left_stick_x - left_stick_y + rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M3, (left_stick_x + left_stick_y - rotation)/denominator);

    }

    public void runTelemetry(){
    }

}
