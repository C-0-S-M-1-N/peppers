package org.firstinspires.ftc.teamcode.Parts;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

/*
* MAP:
* 0 - LF
* 1 - LB
* 2 - RF
* 3 - RB
*
* */
public class MecanumDrive{
    public static boolean Disable = false;
    private Telemetry telemetry;
    public MecanumDrive(Telemetry tele){
        telemetry = tele;
    }

    public void update(double left_stick_y, double left_stick_x,
                       double right_trigger, double left_trigger){
        if(Disable) return;
        double rotation = right_trigger - left_trigger;
        double denominator = Math.max(left_stick_x + left_stick_y + rotation, 1);

        ExpansionHub.setMotorPower(MOTOR_PORTS.M0, (left_stick_x + left_stick_y + rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M1, (left_stick_x - left_stick_y + rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M2, (left_stick_x - left_stick_y - rotation)/denominator);
        ExpansionHub.setMotorPower(MOTOR_PORTS.M3, (left_stick_x + left_stick_y - rotation)/denominator);

    }

    public void runTelemetry(){
    }

}
