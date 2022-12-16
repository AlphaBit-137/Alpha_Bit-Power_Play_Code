package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Slider {

    public DcMotor slider;

    Junctions current_junction = Junctions.Default;

    public enum Junctions{

        High(14000),
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
 //   Pid_Controller PC = new Pid_Controller(0.0085,0.00002,0.0);


  public Motor_Skeleton sliderMotor = new Motor_Skeleton(slider);


    public void init(HardwareMap ahwMap,Gamepad Slider_Gamepad) {
        this.Slider_Gamepad = Slider_Gamepad;
   //     PC.InitTimer();
      sliderMotor.init(ahwMap,"Slider",true);
    }

    public double GetSliderPosition()
    {
        return sliderMotor.MotorCurrentPosition();
    }

    public boolean SliderBusy(){return sliderMotor.isBusy();}

    public void update() {

        if ((Slider_Gamepad.right_bumper /*&& Check() != 1*/) || sliderMotor.isBusy()) {
            sliderMotor.SetPower(1);
        } else if (Slider_Gamepad.left_bumper /*&& Check() != 2*/) {
            sliderMotor.SetPower(-1);
        } else sliderMotor.SetPower(0);


        if(Slider_Gamepad.dpad_up){
            current_junction = Junctions.High;
            sliderMotor.switchToLevel(current_junction.ThisPosition);
        }

        if(Slider_Gamepad.dpad_right)
        {
            current_junction = Junctions.Medium;
            sliderMotor.switchToLevel(current_junction.ThisPosition);
        }

        if(Slider_Gamepad.dpad_down)
        {
            current_junction = Junctions.Low;
            sliderMotor.switchToLevel(current_junction.ThisPosition);
        }

        if(Slider_Gamepad.dpad_left)
        {
            current_junction = Junctions.Default;
            sliderMotor.switchToLevel(current_junction.ThisPosition);
        }


       sliderMotor.StateUpdate();
    }


    public int Check()
    {
        if(sliderMotor.MotorCurrentPosition() >= 16000)return 1;
        else if(sliderMotor.MotorCurrentPosition() <= 0)return 2;
        else return 3;
    }

}