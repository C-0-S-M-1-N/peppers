package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoMotor;

public class Elevator implements Part {
    private Telemetry telemetry;
    private AutoMotor left, right;
    private ElevatorArm Arm;
    public static int maxPos = 950, elevatorPos;

    public Elevator(HardwareMap hm, Telemetry tele){
        telemetry = tele;

        left = new AutoMotor(hm.get(DcMotorEx.class, "left_elevator"));
        right = new AutoMotor(hm.get(DcMotorEx.class, "right_elevator"));
        Arm = new ElevatorArm(hm, tele);

        left.setPower(1);
        right.setPower(1);

        left.resetZeroPosition();
        right.resetZeroPosition();

    }
    @Override
    public void update(){

        Arm.setPrecentageToEnd((float) left.getPosition()/elevatorPos);

        left.update();
        right.update();
        Arm.update();
    }
    public void setPosition(int pos){

    }

    @Override
    public void update_values(){
        // nothing to do here :)
    }
}