package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoMotor;

@Config
public class Elevator implements Part {
    public enum STATES{
        GO_UP,
        GO_DOWN,
        IDLE
    }
    public STATES STATE;
    private Telemetry telemetry;
    private AutoMotor left, right;
    private static int maxPos = 950, elevatorPos;

    public Elevator(HardwareMap hm, Telemetry tele){
        telemetry = tele;

        left = new AutoMotor(hm.get(DcMotorEx.class, "ll"), true);
        right = new AutoMotor(hm.get(DcMotorEx.class, "lr"), false);
        STATE = STATES.IDLE;

        left.setPower(1);
        right.setPower(1);

//        left.resetZeroPosition();
//        right.resetZeroPosition();

    }
    @Override
    public void update(){

        switch (STATE){
            case GO_UP:
                break;
            case GO_DOWN:
                break;
        }

        left.setTargetPosition(elevatorPos);
        right.setTargetPosition(elevatorPos);

        left.update();
        right.update();

        if(left.getVelocity() == 0 && right.getVelocity() == 0) STATE = STATES.IDLE;
    }
    public void setPosition(int pos){
        if(pos > elevatorPos) STATE = STATES.GO_UP;
        if(pos < elevatorPos) STATE = STATES.GO_DOWN;
        else STATE = STATES.IDLE;
        elevatorPos = pos;
    }

    @Override
    public void update_values(){
        // nothing to do here :)
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("position", elevatorPos);
        telemetry.addData("STATE", STATE.toString());
    }
}