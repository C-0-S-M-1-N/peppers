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
    private ElevatorArm Arm;
    public static int maxPos = 950, elevatorPos;

    public Elevator(HardwareMap hm, Telemetry tele){
        telemetry = tele;

        left = new AutoMotor(hm.get(DcMotorEx.class, "ll"), true);
        right = new AutoMotor(hm.get(DcMotorEx.class, "lr"), false);
        Arm = new ElevatorArm(hm, tele);
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
                Arm.setToBackdrop();
                break;
            case GO_DOWN:
                Arm.setToStatic();
                break;
        }

        Arm.setPrecentageToEnd((float) left.getPosition()/elevatorPos);

        left.setTargetPosition(elevatorPos);
        right.setTargetPosition(elevatorPos);

        left.update();
        right.update();

        Arm.update();
        if(left.getVelocity() == 0 && right.getVelocity() == 0){
            STATE = STATES.IDLE;
        }
        telemetry.addData("elevator level", left.getPosition());
        telemetry.addData("state", Arm.STATE.toString());
        telemetry.addData("velocity", right.getVelocity());
    }
    public void setPosition(int pos){
        if(elevatorPos < pos) STATE = STATES.GO_UP;
        else if(elevatorPos > pos) STATE = STATES.GO_DOWN;
        else STATE = STATES.IDLE;
        elevatorPos = pos;
    }

    @Override
    public void update_values(){
        // nothing to do here :)
    }
}