package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public DcMotor arm;

    Positions ArmPosition = Positions.STOP;
    HardwareMap hwMap = null;

    public enum Positions{
        UP,
        DOWN,
        STOP
    }

    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        hwMap = ahwMap;
        arm = hwMap.get(DcMotor.class, "Arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);
    }

    public void Arm_GoToPos(int position){
        arm.setTargetPosition(position);
        arm.setMode((DcMotor.RunMode.RUN_TO_POSITION));
    }


    public void update(){
        switch (ArmPosition){
            case UP:{
                arm.setPower(-0.5);
                break;
            }
            case DOWN:{
                arm.setPower(0.5);
                break;
            }
            case STOP:{
                arm.setPower(0);
                break;
            }
        }
    }

    public void switchToArmUp() {ArmPosition = Positions.UP;}

    public void switchToArmDown() {ArmPosition = Positions.DOWN;}

    public void switchToArmSTOP() {ArmPosition = Positions.STOP;}

    public boolean ArmBUSY() {return arm.isBusy();}

}