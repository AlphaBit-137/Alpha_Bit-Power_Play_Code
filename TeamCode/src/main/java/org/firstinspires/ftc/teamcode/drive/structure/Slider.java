package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;


@Config
public class Slider {
    public double Reference;

    public static double Kp = 0.01;
    public static double Ki = 0.00;
    public static double Kd = 0.0;

    double max_vel = 2000;
    double max_accel = 2000;

    double max_output = 0.55;

    double savedPower = 0;
    boolean firstTime = true;

    double lastPosition;

    ServoClaw sclaw = new ServoClaw();

    ElapsedTime timer = new ElapsedTime();

    public DcMotorEx slider;
    public DcMotorEx slider2;

    Junctions current_junction = Junctions.Default;

    public enum Junctions {

        High(-1500),
        Default(0);

        public int ThisPosition = 0;

        Junctions(int ThisPosition) {
            this.ThisPosition = ThisPosition;
        }

    }

    Gamepad Slider_Gamepad;

    public Motor_Skeleton sliderMotor = new Motor_Skeleton(slider);
    public Motor_Skeleton sliderMotor2 = new Motor_Skeleton(slider2);


    public void init(HardwareMap ahwMap, Gamepad Slider_Gamepad) {
        this.Slider_Gamepad = Slider_Gamepad;

        sliderMotor.init(ahwMap, "Slider", false,false);
        sliderMotor2.init(ahwMap, "Slider2", false,false);

        sliderMotor.setPidCoefs(Kp,Kd,Ki);
        sliderMotor2.setPidCoefs(Kp,Kd,Ki);

        sliderMotor.setMaxAccelandVel(Kp,Kd,Ki,max_accel,max_vel);
    }

    public double GetSliderPosition() {
        return sliderMotor2.MotorCurrentPosition();
    }

    public double GetSliderPower(){return sliderMotor2.GetPower();}

    public void update() {


        SetPidPower(Reference);

        if (Slider_Gamepad.dpad_up) {
            current_junction = Junctions.High;
            setReference(current_junction.ThisPosition);
        }

        if (Slider_Gamepad.dpad_down || Slider_Gamepad.dpad_right || Slider_Gamepad.dpad_left) {
            current_junction = Junctions.Default;
            setReference(current_junction.ThisPosition);
        }

        lastPosition = sliderMotor.MotorCurrentPosition();

    }


    public void SetSliderPower(double power) {
        sliderMotor.SetPower(power);
        sliderMotor2.SetPower(-power);
    }

    public void autoUpdate()
    {
        SetPidPower(Reference);
        lastPosition = sliderMotor.MotorCurrentPosition();
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

        savedPower = sliderMotor2.getPidPower(current_Reference);

       savedPower = addons(savedPower);

        sliderMotor.SetPower(-savedPower);
        sliderMotor2.SetPower(savedPower);
    }

    public double addons(double power)
    {
        if(power > max_output)power = max_output;
        else if(power < -max_output)power = - max_output;


        return power;
    }

    public boolean checkSteady() {
        if((lastPosition == sliderMotor2.MotorCurrentPosition()) && timer.seconds() > 0.5) {
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


