package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.AutoGamepad;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;

@Config
public class Controls {
    public static boolean updateDetected;
    public static boolean Intake, RevIntake,
            ExtendElevator, RetractElevator, ElevatorUp,  ElevatorDown,
            DropLeft, DropRight, Hang;
    public static double HangLevel = 0;
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

    public static RumbleEffectPlay currentState;

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
        DropLeft        = false;
        DropRight       = false;
        RevIntake       = false;
        Hang            = false;

    }

    public void loop(){
        reset();
        gamepad1.update();
        gamepad2.update();
        if(gamepad2.wasPressed.dpad_up)     ExtendElevator  = true;
        if(gamepad2.wasPressed.dpad_down)   RetractElevator = true;
        if(gamepad2.wasPressed.dpad_right)  ElevatorUp      = true;
        if(gamepad2.wasPressed.dpad_left)   ElevatorDown    = true;

        if(gamepad2.right_trigger != 0)     Intake          = true;
        if(gamepad2.left_trigger != 0)      RevIntake       = true;

        if(gamepad1.wasReleased.left_bumper)    DropLeft    = true;
        if(gamepad1.wasReleased.right_bumper)   DropRight   = true;
        if(gamepad2.wasPressed.a){
            HangLevel = -gamepad2.left_stick_y;
        } else HangLevel = 0;

        updateDetected = ExtendElevator || RetractElevator || ElevatorUp || ElevatorDown || Intake
                || DropRight || DropLeft || RevIntake;

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
