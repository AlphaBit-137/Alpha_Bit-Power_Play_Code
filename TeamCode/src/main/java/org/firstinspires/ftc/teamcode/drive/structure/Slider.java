package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Slider {

    Gamepad Slider_Gamepad;

   public Slider(Gamepad Slider_Gamepad)
   {
       this.Slider_Gamepad = Slider_Gamepad;
   }

    public DcMotor slider;

   ServoClaw claw = new ServoClaw();

    Motor_Skeleton skelete = new Motor_Skeleton(slider);


    public void init(HardwareMap ahwMap) {
      skelete.init(ahwMap,"Slider",true);
    }


    public void update(){

       if(Slider_Gamepad.right_bumper)
       {
           skelete.GoTo("up");
       }else if(Slider_Gamepad.left_bumper){
           skelete.GoTo("down");
       }else skelete.GoTo("stop");

       skelete.StateUpdate(0.5,-0.5);
    }



}