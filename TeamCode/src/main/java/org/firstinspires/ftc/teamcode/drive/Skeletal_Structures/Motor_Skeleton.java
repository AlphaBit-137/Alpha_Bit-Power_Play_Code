package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Skeleton {

    DcMotor ThisMotor;

    STATES MotorState = STATES.RESET;

    public boolean GoPos = false;

    int Target;

    HardwareMap hwMap = null;

   public Motor_Skeleton(DcMotor ThisMotor){
        this.ThisMotor = ThisMotor;
    }

    public void init(HardwareMap ahwMap,String MotorName,boolean IsReversed) {

        hwMap = ahwMap;

        ThisMotor = hwMap.get(DcMotor.class, MotorName);

        ThisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ThisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IsReversed(IsReversed);

        ThisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ThisMotor.setPower(0);
    }

    public double MotorCurrentPosition()
    {
        return ThisMotor.getCurrentPosition();
    }

    public void SetTargetPosition(int poz){
        ThisMotor.setTargetPosition(poz);
        ThisMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Reset(){
       ThisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusy(){return ThisMotor.isBusy();}

    public void SetPower(double power)
    {
        ThisMotor.setPower(power);
    }


    enum STATES{
        LVL,
        RESET
    }

    public void StateUpdate(){
        switch (MotorState){
            case LVL:
                Position_Lvl(Target);
                break;
            case RESET:
                Reset();
                break;
        }
        if(!GoPos)switchToReset();
    }

    public void Position_Lvl(int poz2){
        SetTargetPosition(poz2);
         if(!isBusy()){
           GoPos = false;
        }
    }

  public void switchToLevel(int Target){MotorState = STATES.LVL; this.Target = Target; GoPos=true;}
  public void switchToReset(){MotorState = STATES.RESET;}

    public void IsReversed(boolean isreversed)
    {
        if(isreversed){
            ThisMotor.setDirection(DcMotor.Direction.REVERSE);
        }else ThisMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
