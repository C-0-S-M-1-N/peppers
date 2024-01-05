package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.AutoGamepad;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;

@Config
public class Controls {
    public static boolean updateDetected;
    public static boolean Intake,
            ExtendElevator, RetractElevator, ElevatorUp, ElevatorDown,
            RotatePixels, SwapPixels, DropLeft, DropRight;
    private AutoGamepad gamepad1, gamepad2;
    private final RumbleEffects effects;

    public enum RumbleEffectPlay{
        LeftLost, LeftGot,
        RightLost, RightGot,
        FullLoad,
        ENDGAME,
        END,
        NONE;
    }

    public RumbleEffectPlay currentState;

    public Controls(Gamepad gp1, Gamepad gp2){
        gamepad1 = new AutoGamepad(gp1);
        gamepad2 = new AutoGamepad(gp2);
        effects = new RumbleEffects();
        updateDetected = false;
    }

    private void reset(){
        currentState = RumbleEffectPlay.NONE;
        ExtendElevator  = false;
        RetractElevator = false;
        ElevatorUp      = false;
        ElevatorDown    = false;
        Intake          = false;
        RotatePixels    = false;
        SwapPixels      = false;
        DropLeft        = false;
        DropRight       = false;

    }

    public void loop(){
        reset();
        gamepad1.update();
        gamepad2.update();
        if(gamepad2.wasPressed.dpad_up)     ExtendElevator  = true;
        if(gamepad2.wasPressed.dpad_down)   RetractElevator = true;
        if(gamepad2.wasPressed.dpad_right)  ElevatorUp      = true;
        if(gamepad2.wasPressed.dpad_left)   ElevatorDown    = true;

        if(gamepad2.wasPressed.a)           Intake          = true;
        if(gamepad2.wasPressed.x)           RotatePixels    = true;
        if(gamepad2.wasPressed.y)           SwapPixels      = true;

        if(gamepad1.wasReleased.left_bumper)    DropLeft    = true;
        if(gamepad1.wasReleased.right_bumper)   DropRight   = true;

        updateDetected = ExtendElevator || RetractElevator || ElevatorUp || ElevatorDown || Intake || RotatePixels
                || SwapPixels || DropRight || DropLeft;

        switch (currentState){
            case LeftLost:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.LeftLost);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.LeftLost);
                break;
            case LeftGot:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.LeftPixel);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.LeftPixel);
                break;
            case RightLost:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.RightLost);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.RightLost);
                break;
            case RightGot:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.RightPixel);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.RightPixel);
                break;
            case FullLoad:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.fullLoad);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.fullLoad);
                break;
            case ENDGAME:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.ENDGAME);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.ENDGAME);
                break;
            case END:
                gamepad1.gamepad.runRumbleEffect(RumbleEffects.FINISHED);
                gamepad2.gamepad.runRumbleEffect(RumbleEffects.FINISHED);
                break;
            case NONE:
                break;
        }
    }

}
