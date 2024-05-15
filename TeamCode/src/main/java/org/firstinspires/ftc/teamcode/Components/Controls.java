package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.utils.AutoGamepad;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;

import java.util.Set;

@Config
public class Controls {
    public static boolean updateDetected;
    public static boolean Intake, RevIntake,
            ExtendElevator, RetractElevator, ElevatorUp,  ElevatorDown,
            DropLeft, DropRight, Hang, Avion, ResetTourret,
            DownElevator,
            ResetElevator, ManualMode, PokeMode, SetOuttakeToPurplePlacing, rotateLeft, rotateRight;
    public static double HangLevel = 0;
    private static AutoGamepad gamepad1;
    private static AutoGamepad gamepad2;
    private final RumbleEffects effects;

       public static boolean LeftLost, LeftGot,
        RightLost, RightGot,
        FullLoad, ENDGAME, END, NONE;

    public Controls(Gamepad gp1, Gamepad gp2){
        gamepad1 = new AutoGamepad(gp1);
        gamepad2 = new AutoGamepad(gp2);
        effects = new RumbleEffects();
        updateDetected = false;
    }

    private void reset(){
        ExtendElevator  = false;
        RetractElevator = false;
        ElevatorUp      = false;
        ElevatorDown    = false;
        Intake          = false;
        DropLeft        = false;
        DropRight       = false;
        RevIntake       = false;
        Hang            = false;
        Avion           = false;
        ResetTourret    = false;
        ResetElevator   = false;
        DownElevator    = false;
        ManualMode      = false;
        PokeMode        = false;
        SetOuttakeToPurplePlacing = false;
        rotateLeft      = false;
        rotateRight     = false;
    }

    public void loop(){
        reset();
        gamepad1.update();
        gamepad2.update();
        if(gamepad2.wasPressed.dpad_up)     ExtendElevator  = true;
        if(gamepad2.wasPressed.dpad_down)   RetractElevator = true;
        if(gamepad2.wasPressed.dpad_right)  ElevatorUp      = true;
        if(gamepad2.wasPressed.dpad_left)   ElevatorDown    = true;
        if(gamepad2.wasPressed.a) Hang = true;
        if(gamepad2.wasPressed.left_bumper) rotateLeft = true;
        if(gamepad2.wasPressed.right_bumper) rotateRight = true;

        if(gamepad2.right_trigger >= 0.7)     Intake          = true;
        if(gamepad2.left_trigger >= 0.7)      RevIntake       = true;

        if(gamepad1.wasPressed.left_bumper)    DropLeft    = true;
        if(gamepad1.wasPressed.right_bumper)   DropRight   = true;
        if(gamepad2.gamepad.a){
            HangLevel = gamepad2.gamepad.left_stick_y;
        } else HangLevel = 0;
        if(gamepad2.wasPressed.x) Avion = true;
        if(gamepad2.wasPressed.triangle) ResetTourret = true;
        if(gamepad2.wasReleased.circle)  ResetElevator = true;
        if(gamepad2.wasPressed.circle) DownElevator = true;
        if(gamepad2.wasPressed.left_bumper && gamepad2.wasPressed.right_bumper) ManualMode = true;
        if(gamepad1.wasPressed.circle) PokeMode = true;

        updateDetected = ExtendElevator || RetractElevator || ElevatorUp || ElevatorDown || Intake
                || DropRight || DropLeft || RevIntake;

        playEffects();
    }

    private static void playEffects(){
        if(FullLoad){
            gamepad1.gamepad.runRumbleEffect(RumbleEffects.fullLoad);
            gamepad2.gamepad.runRumbleEffect(RumbleEffects.fullLoad);
        }
        else if(LeftGot){
            gamepad1.gamepad.runRumbleEffect(RumbleEffects.LeftPixel);
            gamepad2.gamepad.runRumbleEffect(RumbleEffects.LeftPixel);
        }
        else if(RightGot){
            gamepad1.gamepad.runRumbleEffect(RumbleEffects.RightPixel);
            gamepad2.gamepad.runRumbleEffect(RumbleEffects.RightPixel);
        }

        if(LeftLost){
            gamepad1.gamepad.runRumbleEffect(RumbleEffects.LeftLost);
            gamepad2.gamepad.runRumbleEffect(RumbleEffects.LeftLost);
        }
        if(RightLost){
            gamepad1.gamepad.runRumbleEffect(RumbleEffects.RightLost);
            gamepad2.gamepad.runRumbleEffect(RumbleEffects.RightLost);
        }

        FullLoad = false;
        LeftGot = false;
        LeftLost = false;
        RightLost = false;
        RightGot = false;
    }

}
