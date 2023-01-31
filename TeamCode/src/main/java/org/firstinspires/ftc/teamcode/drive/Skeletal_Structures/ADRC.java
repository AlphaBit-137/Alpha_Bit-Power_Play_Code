package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.util.ElapsedTime;


public class ADRC {

    private ElapsedTime period  = new ElapsedTime();


    private double kp;
    private double ki;
    private double kd;
    private double error;
    private double lastError;
    private double integral;
    private double derivative;
    private double output;
    private double maxOutput;


    private double beta;
    private double d;
    private double previousOutput;


    public  ADRC (double kp, double ki, double kd,
                  double maxOutput, double beta, double d) {


        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.maxOutput = maxOutput;
        this.beta = beta;
        this.d = d;

    }

    public double getPower(double state,double setpoint) {

        error = setpoint - state;

        integral += error * period.seconds();

        derivative = (error - lastError) / period.seconds();

        output = kp * error + ki * integral + kd * derivative;

        output = output + beta * (output - previousOutput) + d * (error - lastError);

        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < -maxOutput) {
            output = -maxOutput;
        }

        lastError = error;
        previousOutput = output;


        period.reset();

        return output;
    }
}
