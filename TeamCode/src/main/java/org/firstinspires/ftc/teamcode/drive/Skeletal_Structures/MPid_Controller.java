package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.util.ElapsedTime;
//Made this to use PID with mp, made in a seperate class since you are using mp only if your normal pid doesn't work as good
public class MPid_Controller {

    MotionProfile MP = new MotionProfile();
    ElapsedTime timer = new ElapsedTime();

    double Kp = 0.0;
    double Kd = 0.0;
    double Ki = 0.0;

    public boolean IsStarted = false;

    double maxAccel = 0.0;
    double maxVel = 0.0;

    double IntegralSum = 0.0;
    double LastError = 0.0;
    double LastReference = 0.0;

    public MPid_Controller(double Kp, double Ki, double Kd,double maxAccel,double maxVel)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }

    public double returnPower(double reference, double state , double velocity){

        double time = getTime();

        double error = reference - state;

        velocity = normalize(velocity);

        double InstantError = MP.motion_profile(maxAccel,maxVel,error,error/velocity);

        if(LastReference != reference)
        {
            IntegralSum = 0;
        }

        IntegralSum += InstantError * time;

        double derivative = (InstantError - LastError) / time;

        LastError = InstantError;
        LastReference = reference;
        timer.reset();

        double outpput = (InstantError * Kp) + (derivative * Kd) + (IntegralSum * Ki);


        return outpput;
    }

    public double returnPmotion(double reference,double state,double velocity)
    {
        double error = reference - state;

        velocity = normalize(velocity);

        double InstantError = MP.motion_profile(maxAccel,maxVel,error,error/velocity);

        return InstantError;
    }

    double normalize(double vel)
    {
        if(vel == 0)return 1;
        else return vel;
    }

    double getTime()
    {
        if(!IsStarted)
        {
            timer.reset();
            IsStarted = true;
        }
        double time = timer.seconds();
        return time;
    }

}
