package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ResourceBundle;

/*
* MAP:
* 3 - INTAKE
* */
@Config
public class Intake implements Part {
    public static boolean Disabled = false;
    public enum STATES{
        IDLE,
        REVERSE,
        FORWARD
    }
    public STATES STATE;
    public static double maxTrashHold = 1200;
    public static double ground = 145;
    private double usedCurrent = 0;
    public AutoServo servo;

    NanoClock clock;
    private double grippersHaveTime = 0;
    private boolean grippersHave = false;
    public static double[] stackPositions = {73, 80, 105, 105, 105};

    public void setPixelStackPosition(int level){
        if(level > 4) level = 4;
        ground = stackPositions[level];
    }

    public Intake(){
        STATE = STATES.IDLE;
        ControlHub.setMotorDirection(MOTOR_PORTS.M2, DcMotorSimple.Direction.REVERSE);
        servo = new AutoServo(SERVO_PORTS.S5, 0,
                true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        servo.setAngle(30);
        servo.update();
        clock = NanoClock.system();
        setPixelStackPosition(4);
    }
    private boolean close = false;
    private ElapsedTime manualTimer = new ElapsedTime();
    STATES prevState = STATES.IDLE;
    private void manualUpdate(){
        STATE = STATES.IDLE;
        if(Controls.Intake){
            STATE = STATES.FORWARD;
            prevState = STATES.FORWARD;
            manualTimer.reset();
        }
        if(Controls.RevIntake){
            STATE = STATES.REVERSE;
            prevState = STATES.REVERSE;
            manualTimer.reset();
        }

        close = manualTimer.seconds() >= 0.3;

        if(close){
            OutTake.leftGripper.close();
            OutTake.rightGripper.close();
        } else {
            OutTake.rightGripper.open();
            OutTake.leftGripper.open();
        }

        switch (STATE){
            case IDLE:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                servo.setAngle(30);
                break;
            case FORWARD:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                servo.setAngle(ground);
                break;
            case REVERSE:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                servo.setAngle(30);
                break;
        }


        servo.update();

    }
    @Override
    public void update(){
        if(Disabled) return;
        if(Grippers.manualMode) manualUpdate();

        if(!OutTake.fullPixel()) {
            grippersHaveTime = 0;
            grippersHave = false;
        } else if(grippersHaveTime == 0) {
            grippersHaveTime = clock.seconds();
        } else if(clock.seconds() - grippersHaveTime > 0) {
            grippersHave = true;
        }
        if(usedCurrent > maxTrashHold){
//            STATE = STATES.REVERSE;
        }
        if(Controls.Intake && !grippersHave){
            STATE = STATES.FORWARD;
        }
        if(Controls.RevIntake || (grippersHave && Controls.Intake)){
            STATE = STATES.REVERSE;
        }
        if(!Controls.RevIntake && !Controls.Intake) STATE = STATES.IDLE;

        switch (STATE){
            case IDLE:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                servo.setAngle(30);
                break;
            case FORWARD:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                servo.setAngle(ground);
                break;
            case REVERSE:
                ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                servo.setAngle(30);
                break;
        }
        servo.update();

    }
    @Override
    public void update_values(){
        usedCurrent = ControlHub.getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.MILLIAMPS);
    }
    @Override
    public void runTelemetry(){}
}
