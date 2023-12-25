package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.AutoGamepad;

@Config
public class Controls {
    public static boolean Intake,
            ExtendElevator, RetractElevator, ElevatorUp, ElevatorDown,
            RotatePixels, SwapPixels, DropLeft, DropRight;
    private AutoGamepad gamepad1, gamepad2;

    public Controls(Gamepad gp1, Gamepad gp2){
        gamepad1 = new AutoGamepad(gp1);
        gamepad2 = new AutoGamepad(gp2);
    }

    private void applyRumbleEffects(){

    }
    private void reset(){

    }

    public void loop(){
        if(gamepad2.wasPressed.dpad_up)     ExtendElevator  = true;
        if(gamepad2.wasPressed.dpad_down)   RetractElevator = true;
        if(gamepad2.wasPressed.dpad_right)  ElevatorUp      = true;
        if(gamepad2.wasPressed.dpad_left)   ElevatorDown    = true;

        if(gamepad2.wasPressed.a)           Intake          = true;
        if(gamepad2.wasPressed.x)           RotatePixels    = true;
        if(gamepad2.wasPressed.y)           SwapPixels      = true;

        if(gamepad1.wasReleased.left_bumper)    DropLeft    = true;
        if(gamepad1.wasReleased.right_bumper)   DropRight   = true;
    }

}
