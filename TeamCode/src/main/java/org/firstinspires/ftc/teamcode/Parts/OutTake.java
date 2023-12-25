package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoSensor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class OutTake implements Part {
    public static boolean disable = false;
    public enum STATES{
        EXTEND_TRIGGER,
        EXTEND,
        RETRACT,
        RETRACT_TRIGGER,
        LEVEL_UP,
        LEVEL_DOWN,
        IDLE;
        public double prevElevatorLevel = 5;
        public double currentElevatorLevel = 0;
        public double step = 950/11;
    }

    private static double extendArm = 0.75;

    public STATES STATE;
    private Telemetry telemetry;
    private static Controls controls;

    private static Elevator elevator;
    private static ElevatorArm arm;
    private static Grippers leftClaw, rightClaw;
    private static PixelBed pixelBed;

    public OutTake(HardwareMap hm, Controls c, Telemetry tele){
        telemetry = tele;
        controls = c;

        elevator = new Elevator(hm, tele);
        arm = new ElevatorArm(hm, tele);
        pixelBed = new PixelBed(hm, tele);

        leftClaw = new Grippers(new AutoServo(hm.get(Servo.class, "leftclaw"), false, 0, AutoServo.type.GOBILDA),
                                new AutoSensor(hm.get(RevColorSensorV3.class, "sensorLeft"), 20));
        rightClaw = new Grippers(new AutoServo(hm.get(Servo.class, "rightclaw"), false, 0, AutoServo.type.GOBILDA),
                new AutoSensor(hm.get(RevColorSensorV3.class, "sensorRight"), 20));

    }

    private void handleControls(){

    }

    @Override
    public void update(){
        if(disable) return;
        switch (STATE){
            case EXTEND_TRIGGER:
                arm.setPosition(extendArm);
                elevator.setPosition((int) (Math.asin(arm.getPosition())*10));
                if(arm.getPosition() >= extendArm - 0.01 &&
                    arm.getPosition() <= extendArm + 0.01){
                    STATE = STATES.EXTEND;
                }
                break;
            case EXTEND:
                elevator.setPosition((int) (STATE.prevElevatorLevel*STATE.step));
                if(elevator.STATE == Elevator.STATES.IDLE){
                    STATE = STATES.IDLE;
                }
                break;
            case IDLE:
                break;
            case LEVEL_UP:
                STATE.currentElevatorLevel ++;
                if(STATE.currentElevatorLevel > 11) STATE.currentElevatorLevel = 11;

                elevator.setPosition((int)(STATE.currentElevatorLevel * STATE.step));

                break;
            case LEVEL_DOWN:
                STATE.currentElevatorLevel --;
                if(STATE.currentElevatorLevel < 0) STATE.currentElevatorLevel = 0;

                elevator.setPosition((int)(STATE.currentElevatorLevel * STATE.step));

                break;
            case RETRACT_TRIGGER:
                STATE.prevElevatorLevel = STATE.currentElevatorLevel;

                arm.setPosition(0);
                elevator.setPosition((int) (Math.asin(arm.getPosition())*10));

                if(arm.getPosition() <= 0.01){
                    STATE = STATES.RETRACT;
                }
                break;
            case RETRACT:
                elevator.setPosition(0);
                break;
        }
        arm.update();
        elevator.update();
        pixelBed.update();
        leftClaw.update();
        rightClaw.update();
    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        elevator.runTelemetry();
        arm.runTelemetry();
        rightClaw.update();
        leftClaw.update();
    }
}
