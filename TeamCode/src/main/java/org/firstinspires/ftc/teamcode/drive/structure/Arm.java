package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.MPid_Controller;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Arm {
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

    Pid_Controller PID = new Pid_Controller(Kp,Ki,Kd);
    MPid_Controller MPID = new MPid_Controller(Kp,Ki,Kd,max_accel,max_vel);

    public DcMotorEx arm;

    Gamepad Arm_Gamepad;

    public Motor_Skeleton ArmMotor = new Motor_Skeleton(arm);

    public void init(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor.init(ahwMap,"Arm",true,false);
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

            if(firstTime) {
                timer.reset();
                firstTime = false;
            }

            SetPidPower(Reeference);
        }

        if(Arm_Gamepad.dpad_up)
        {
            setReference(2300);
        }

        if(Arm_Gamepad.dpad_down)
        {
            setReference(75);
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

    public void SetMPidPower(double reference)
    {
        ArmMotor.SetPower(-MPID.returnPower(reference,ArmMotor.MotorCurrentPosition(),ArmMotor.GetVelocity()));
    }

    public void SetPidPower(double reference)
    {

        if(!checkSteady())
        {
            savedPower = -PID.returnPower(reference,ArmMotor.MotorCurrentPosition());;
        }

        savedPower = addons(savedPower);

        ArmMotor.SetPower(savedPower);
    }

    double addons(double pow)
    {
        if((Math.abs(ArmMotor.MotorCurrentPosition()) < 800 && Reeference < 1000) || (Math.abs(ArmMotor.MotorCurrentPosition()) > 1500 && Reeference > 1000))
        {
            max_output = 0.45;
        }else max_output = 0.9;


        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;


        return pow;
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
