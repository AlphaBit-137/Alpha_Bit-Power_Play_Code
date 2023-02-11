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

    public double normalizer;

    double minimum;

    boolean isLinear = false;

    enum BusyStates{
        isBusy,
        isBusy2,
        notBusy
    }

    BusyStates bs = BusyStates.notBusy;

    Pid_Controller pid;
    MPid_Controller mpid;
    FeedForward_Control ff ;
    Voltage_Sensor vs = new Voltage_Sensor();

    public void init(HardwareMap ahwMap,String MotorName,boolean IsReversed,boolean using_encoders) {

        ThisMotor = ahwMap.get(DcMotorEx.class, MotorName);

        ThisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vs.init(ahwMap,normalizer);

        RUN_WITH_ENCODERS(using_encoders);

        IsReversed(IsReversed);

        ThisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ThisMotor.setPower(0);
    }

    public double MotorCurrentPosition()
    {
        return ThisMotor.getCurrentPosition();
    }

    public void SetPower(double power)
    {
        ThisMotor.setPower(power);
    }

    public double GetPower(){return ThisMotor.getPower();}

    public void SetVelocity(double velocity)
    {
        ThisMotor.setVelocity(velocity);
    }

    public double GetVelocity()
    {
       return ThisMotor.getVelocity();
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

    public void setPidCoefs(double Kp,double Kd,double Ki)
    {
        pid = new Pid_Controller(Kp,Kd,Ki);
    }

    public void setMaxAccelandVel(double maxVel, double maxAccel,double Kp,double Kd,double Ki)
    {
       mpid = new MPid_Controller(Kp,Ki,Kd,maxAccel,maxVel);
    }

    public double powerConstraints(double maxOutput,double power)
    {
        if(power > maxOutput)power = maxOutput;
        else if(power < -maxOutput)power = -maxOutput;

        return power;
    }

    public void getNormalizer(double normalizer){
       this.normalizer = normalizer;
    }

    public void setPidPower(double Reference)
    {
        double power = pid.returnPower(Reference,ThisMotor.getCurrentPosition());
        ThisMotor.setPower(power);
    }

    public double getPidPower(double Reference)
    {
        return pid.returnPower(Reference,MotorCurrentPosition());
    }

    public void setMPidPower(double reference)
    {
        double power = mpid.returnPower(reference,ThisMotor.getCurrentPosition(),ThisMotor.getVelocity());
        ThisMotor.setPower(power);
    }

    public double returnMpidPower(double reference)
    {
        return mpid.returnPower(reference,ThisMotor.getCurrentPosition(),ThisMotor.getVelocity());
    }

     /** Voltage related stuff*/

    public double getCompensatedPower(double curPower)
    {
        return curPower * vs.GetCompensation();
    }

    public void setCompensatedPower(double power)
    {
        ThisMotor.setPower(getCompensatedPower(power));
    }

    /**States related stuff*/

    public void setMinimum(double minimum)
    {
        this.minimum = minimum;
    }


    public boolean isBusy(boolean Linear,double reference)
    {
        if(Linear)bs = BusyStates.isBusy;
        else bs = BusyStates.isBusy2;

        switch (bs)
        {
            case isBusy:
                return Math.abs(reference - ThisMotor.getCurrentPosition()) == 0;
            case isBusy2:
                return Math.abs(reference - ThisMotor.getCurrentPosition()) < minimum;
        }

        return false;
    }

    public boolean isSteady(double lastPosition)
    {
                return Math.abs(ThisMotor.getCurrentPosition() - lastPosition) == 0;
    }
}
