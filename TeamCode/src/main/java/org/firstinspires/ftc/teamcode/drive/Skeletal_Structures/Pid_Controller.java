package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Pid_Controller {

    ElapsedTime timer = new ElapsedTime();

    boolean IsStarted = false;

    double timp;

    public double LastError = 0;
    public double IntegralSum = 0;
    public double Last_Reference = 0;

    public double Kp = 0.0;
    public double Ki = 0.0;
    public double Kd = 0.0;

    public Pid_Controller(double Kp, double Ki, double Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }


    double a = 0.9;
  double  previousFilterEstimate = 0;
  double  currentFilterEstimate = 0;

    public double returnPower(double reference, double state){

        timp = returnTimer();

        double error = reference - state;

        if(Last_Reference != reference)
        {
            IntegralSum = 0;
        }

        IntegralSum += error * timp;

       double errorChange = (error - LastError);

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

       double derivative = currentFilterEstimate / timp;

        LastError = error;

        double outpput = (error * Kp) + (derivative * Kd) + (IntegralSum * Ki);

        Last_Reference = reference;

        timer.reset();

        return outpput;
    }

    public double returnTimer()
    {
        if(!IsStarted) {
            IsStarted = true;
            timer.reset();
        }
        double time = timer.seconds();
        return time;
    }

}