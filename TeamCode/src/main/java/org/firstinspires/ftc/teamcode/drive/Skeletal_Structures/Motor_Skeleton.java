package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Skeleton {

    DcMotorEx ThisMotor;

   public Motor_Skeleton(DcMotorEx ThisMotor){
        this.ThisMotor = ThisMotor;
    }

    public void init(HardwareMap ahwMap,String MotorName,boolean IsReversed,boolean using_encoders) {

        ThisMotor = ahwMap.get(DcMotorEx.class, MotorName);

        ThisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RUN_WITH_ENCODERS(using_encoders);

        IsReversed(IsReversed);

        ThisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ThisMotor.setPower(0);
    }

    public double MotorCurrentPosition()
    {
        return ThisMotor.getCurrentPosition();
    }

    public boolean isBusy(){return ThisMotor.isBusy();}

    public void SetPower(double power)
    {
        ThisMotor.setPower(power);
    }

    public double GetPower(){return ThisMotor.getPower();}

    public void SetVelocity(double velocity)
    {
        ThisMotor.setVelocity(velocity);
    }

    public void GetVelocity()
    {
        ThisMotor.getVelocity();
    }

    public void IsReversed(boolean isreversed)
    {
        if(isreversed){
            ThisMotor.setDirection(DcMotor.Direction.REVERSE);
        }else ThisMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void RUN_WITH_ENCODERS(boolean encoders)
    {
        if(encoders)
        {
            ThisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else ThisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
