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

    public static boolean IntakeAck, RevIntakeAck,
            ExtendElevatorAck, RetractElevatorAck, ElevatorUpAck,  ElevatorDownAck,
            DropLeftAck, DropRightAck, HangAck, AvionAck, ResetTourretAck,
            DownElevatorAck,
            ResetElevatorAck, ManualModeAck, PokeModeAck, SetOuttakeToPurplePlacingAck, rotateLeftAck, rotateRightAck;

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
        if(ExtendElevatorAck) {
            ExtendElevator = false;
            ExtendElevatorAck = false;
        }
        if(RetractElevatorAck) {
            RetractElevator = false;
            RetractElevatorAck = false;
        }
        if(ElevatorUpAck) {
            ElevatorUp = false;
            ElevatorUpAck = false;
        }
        if(ElevatorDownAck) {
            ElevatorDown = false;
            ElevatorDownAck = false;
        }
        Intake = false;
        if(DropLeftAck) {
            DropLeft = false;
            DropLeftAck = false;
        }
        if(DropRightAck) {
            DropRight = false;
            DropRightAck = false;
        }
        RevIntake = false;
        if(HangAck) {
            Hang = false;
            HangAck = false;
        }
        if(AvionAck) {
            Avion = false;
            AvionAck = false;
        }
        if(ResetTourretAck) {
            ResetTourret = false;
            ResetTourretAck = false;
        }
        if(ResetElevatorAck) {
            ResetElevator = false;
            ResetElevatorAck = false;
        }
        if(DownElevatorAck) {
            DownElevator = false;
            DownElevatorAck = false;
        }
        if(ManualModeAck) {
            ManualMode = false;
            ManualModeAck = false;
        }
        if(PokeModeAck) {
            PokeMode = false;
            PokeModeAck = false;
        }
        if(SetOuttakeToPurplePlacingAck) {
            SetOuttakeToPurplePlacing = false;
            SetOuttakeToPurplePlacingAck = false;
        }
        if(rotateLeftAck) {
            rotateLeft = false;
            rotateLeftAck = false;
        }
        if(rotateRightAck) {
            rotateRight = false;
            rotateRightAck = false;
        }
    }

    public void loop(){
        reset();
        gamepad1.update();
        gamepad2.update();
        if(gamepad2.wasPressed.square && gamepad2.wasPressed.right_bumper && gamepad2.wasPressed.triangle) SetOuttakeToPurplePlacing = true;
        else if(gamepad2.wasPressed.dpad_up) {
            ExtendElevator = true;
            ExtendElevatorAck = false;
        }
        else if(gamepad2.wasPressed.dpad_down) {
            RetractElevator = true;
            RetractElevatorAck = false;
        }
        else if(gamepad2.wasPressed.dpad_right) {
            ElevatorUp = true;
            ElevatorUpAck = false;
        }
        else if(gamepad2.wasPressed.dpad_left) {
            ElevatorDown = true;
            ElevatorDownAck = false;
        }
        else if(gamepad2.wasPressed.a) {
            Hang = true;
            HangAck = false;
        }
        else if(gamepad2.wasPressed.left_bumper) {
            rotateLeft = true;
            rotateLeftAck = false;
        }
        else if(gamepad2.wasPressed.right_bumper) rotateRight = true;

        else if(gamepad2.right_trigger >= 0.7) {
            Intake = true;
            IntakeAck = false;
        }
        else if(gamepad2.left_trigger >= 0.7) {
            RevIntake = true;
            RevIntakeAck = false;
        }

        if(gamepad1.wasPressed.left_bumper) {
            DropLeft = true;
            DropLeftAck = false;
        }
        if(gamepad1.wasPressed.right_bumper) {
            DropRight = true;
            DropRightAck = false;
        }
        if(gamepad2.wasPressed.x) {
            Avion = true;
            AvionAck = false;
        }
        if(gamepad2.wasPressed.triangle) {
            ResetTourret = true;
            ResetTourretAck = false;
        }
        if(gamepad2.wasReleased.circle) {
            ResetElevator = true;
            ResetElevatorAck = false;
        }
        if(gamepad2.wasPressed.circle) {
            DownElevator = true;
            DownElevatorAck = false;
        }
        if(gamepad2.wasPressed.left_bumper && gamepad2.wasPressed.right_bumper) {
            ManualMode = true;
            ManualModeAck = false;
        }
        if(gamepad1.wasPressed.circle) {
            PokeMode = true;
            PokeModeAck = false;
        }

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
