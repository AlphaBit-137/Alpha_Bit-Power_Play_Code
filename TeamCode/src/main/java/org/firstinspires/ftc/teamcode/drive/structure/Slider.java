package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Slider {

    public double Reference;

    public static double Kp = 0.0085;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public double max_output = 0.85;

    Pid_Controller PID = new Pid_Controller(Kp,Ki,Kd);

    public DcMotorEx slider;
    public DcMotorEx slider2;


    Junctions current_junction = Junctions.Default;

    public enum Junctions{

        High(3200),
        Medium(7000),
        Low(4000),
        Ground(1000),
        Default(0);

        public int ThisPosition = 0;

        Junctions(int ThisPosition)
        {
            this.ThisPosition = ThisPosition;
        }

    }

    Gamepad Slider_Gamepad;

    public Motor_Skeleton sliderMotor = new Motor_Skeleton(slider);
    public Motor_Skeleton sliderMotor2 = new Motor_Skeleton(slider2);


    public void init(HardwareMap ahwMap,Gamepad Slider_Gamepad) {
        this.Slider_Gamepad = Slider_Gamepad;

        sliderMotor.init(ahwMap,"Slider",false,false);
        sliderMotor2.init(ahwMap,"Slider2",false,false);
    }

    public double GetSliderPosition()
    {
        return sliderMotor.MotorCurrentPosition();
    }

    public double GetSlider2Position(){return sliderMotor2.MotorCurrentPosition();}

    public void update() {


        if(Slider_Gamepad.right_bumper)
        {
            Reference = sliderMotor.MotorCurrentPosition();
            SetSliderPower(1);
        }else if(Slider_Gamepad.left_bumper)
        {
            Reference = sliderMotor.MotorCurrentPosition();
            SetSliderPower(-1);
        }else SetPidPower(Reference);

        if(Slider_Gamepad.dpad_up)
        {
            Reference = 1500;
        }

        if(Slider_Gamepad.dpad_down)
        {
            Reference = 0;
        }

    }

    public void SetSliderPower(double power)
    {
        sliderMotor.SetPower(power);
        sliderMotor2.SetPower(-power);
    }

    public void SetPidPower(double current_Reference)
    {
        double power = PID.returnPower(current_Reference,sliderMotor.MotorCurrentPosition());

        power = addons(power);

        sliderMotor.SetPower(power);
        sliderMotor2.SetPower(-power);
    }

    double addons(double pow)
    {

        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;


        return pow;
    }

}


