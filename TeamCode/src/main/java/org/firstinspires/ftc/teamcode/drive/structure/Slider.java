package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Slider {
    public double Reference;

    public static double Kp = 0.0085;
    public static double Ki = 0.00002;
    public static double Kd = 0.0;

    double savedPower = 0;
    boolean firstTime = true;

    double lastPosition;

    ServoClaw sclaw = new ServoClaw();

    ElapsedTime timer = new ElapsedTime();


    Pid_Controller PID = new Pid_Controller(Kp, Ki, Kd);

    public DcMotorEx slider;
    public DcMotorEx slider2;


    Junctions current_junction = Junctions.Default;

    public enum Junctions {

        High(1500),
        Default(0);

        public int ThisPosition = 0;

        Junctions(int ThisPosition) {
            this.ThisPosition = ThisPosition;
        }

    }

    Gamepad Slider_Gamepad;
    //   Pid_Controller PC = new Pid_Controller(0.0085,0.00002,0.0);


    public Motor_Skeleton sliderMotor = new Motor_Skeleton(slider);
    public Motor_Skeleton sliderMotor2 = new Motor_Skeleton(slider2);


    public void init(HardwareMap ahwMap, Gamepad Slider_Gamepad) {
        this.Slider_Gamepad = Slider_Gamepad;

        sliderMotor.init(ahwMap, "Slider", false,false);
        sliderMotor2.init(ahwMap, "Slider2", false,false);
    }

    public double GetSliderPosition() {
        return sliderMotor.MotorCurrentPosition();
    }


    public void update() {


        if (Slider_Gamepad.right_bumper) {

            setReference(sliderMotor.MotorCurrentPosition());
            SetSliderPower(1);
        } else if (Slider_Gamepad.left_bumper) {

            setReference(sliderMotor.MotorCurrentPosition());
            SetSliderPower(-1);
        }else{
            SetPidPower(Reference);
        }

        if (Slider_Gamepad.dpad_up) {
            current_junction = Junctions.High;
            setReference(current_junction.ThisPosition);
        }

        if (Slider_Gamepad.dpad_down) {
            current_junction = Junctions.Default;
            setReference(current_junction.ThisPosition);
        }

        lastPosition = sliderMotor.MotorCurrentPosition();

    }


    public void SetSliderPower(double power) {
        sliderMotor.SetPower(power);
        sliderMotor2.SetPower(-power);
    }

    public double getSlider2Power() {
        return sliderMotor2.GetPower();
    }

    public double getSliderPower() {
        return sliderMotor.GetPower();
    }

    public int checker() {
        if (sliderMotor.MotorCurrentPosition() < 0) return 1;
        else return 0;
    }

    public void SetPidPower(double current_Reference) {

        if(firstTime) {
            timer.reset();
            firstTime = false;
        }

        if(!checkSteady())
        {
            savedPower = PID.returnPower(current_Reference,sliderMotor.MotorCurrentPosition());;
        }

        sliderMotor.SetPower(savedPower);
        sliderMotor2.SetPower(-savedPower);
    }

    public boolean checkSteady() {
        if((lastPosition == sliderMotor.MotorCurrentPosition()) && timer.seconds() > 0.5) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setReference(double value)
    {
        Reference = value;
        firstTime = true;
    }

}


