package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

import java.util.ResourceBundle;

@Config
public class Avion {
    public static double positionArmat = 0.65, positionShoot = 0.82;
    public boolean shoot = false;
    public Avion(){

    }
    public void update(){
        ControlHub.setServoPosition(SERVO_PORTS.S0, shoot ? positionArmat : positionShoot);
        if(Controls.Avion){
            shoot = !shoot;
        }
    }
}
