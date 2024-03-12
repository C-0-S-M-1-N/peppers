package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

import java.util.ResourceBundle;

@Config
public class Avion {
    public static double armatPos = 0.70, shootPos = 0.4333;
    public boolean shoot = false;
    public Avion(){

    }
    public void update(){
        ExpansionHub.setServoPosition(SERVO_PORTS.S5, shoot ? shootPos : armatPos);
        if(Controls.Avion){
            shoot = !shoot;
        }
    }
}
