package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Arm {

    public DcMotorEx arm;

    public Motor_Skeleton ArmMotor = new Motor_Skeleton(arm);

    Gamepad Arm_Gamepad;

    double Kp = 0.005;
    double Ki = 0.0;
    double Kd = 0.0;

    double max_accel = 1000;
    double max_vel = 1000;

    double max_output = 0.85;

    public double Reeference = 0;

    ElapsedTime timer = new ElapsedTime();

    double savedPower = 0;
    boolean firstTime = true;

    double lastPosition;



    public void init(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor.init(ahwMap,"Arm",true,false);

        ArmMotor.setPidCoefs(Kp,Kd,Ki);
    }

    public void update()
    {
        if(Arm_Gamepad.right_trigger != 0 && Arm_Gamepad.left_trigger == 0)
        {
            setReference(ArmMotor.MotorCurrentPosition());
            SetPower(-0.7);
        }else if(Arm_Gamepad.left_trigger != 0 && Arm_Gamepad.right_trigger == 0)
        {
            setReference(ArmMotor.MotorCurrentPosition());
            SetPower(0.7);
        }else{
            SetPidPower(Reeference);
        }

        if(Arm_Gamepad.dpad_up)
        {
            setReference(2270);
        }

        if(Arm_Gamepad.dpad_left)
        {
            setReference(1100);
        }

        if(Arm_Gamepad.x)
        {
            //setReference(550);
        }

        if(Arm_Gamepad.dpad_right)
        {
            setReference(2000);
        }

        if(Arm_Gamepad.dpad_down)
        {
            setReference(-50);
        }

        lastPosition = ArmMotor.MotorCurrentPosition();

    }

    public double getArmPos()
    {
        return ArmMotor.MotorCurrentPosition();
    }

    public void SetPower(double power) {
        ArmMotor.SetPower(power);
    }


    public void SetPidPower(double reference)
    {
        if(firstTime) {
            timer.reset();
            firstTime = false;
        }

        if(!checkSteady())
        {
          //  savedPower = -PID.returnPower(reference,ArmMotor.MotorCurrentPosition());;
            savedPower = ArmMotor.getPidPower(reference);
        }

        savedPower = addons(savedPower);

        ArmMotor.SetPower(savedPower);
    }

    double addons(double pow)
    {
        if((Math.abs(ArmMotor.MotorCurrentPosition()) < 800 && Reeference < 1000) || (Math.abs(ArmMotor.MotorCurrentPosition()) > 1500 && Reeference > 1000))
        {
            max_output = 0.6;
        }else max_output = 0.9;


        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;


        return pow;
    }

    public double getReference()
    {
        return Reeference;
    }

    public double getArmPower()
    {
        return ArmMotor.GetPower();
    }

    public boolean checkSteady() {
        if((lastPosition == ArmMotor.MotorCurrentPosition()) && timer.seconds() > 0.3) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setReference(double value)
    {
        Reeference = value;
        firstTime = true;
    }



}
