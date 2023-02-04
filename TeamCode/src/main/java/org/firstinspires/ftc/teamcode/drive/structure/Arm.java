package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.MPid_Controller;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Arm {

    double Kp = 0.005;
    double Ki = 0.0;
    double Kd = 0.0;

    double max_accel = 1000;
    double max_vel = 1000;

    double max_output = 0.9;

    double Reeference = 75;

    double LastReference;

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
            SetPower(0.3);
        }else if(Arm_Gamepad.left_trigger != 0 && Arm_Gamepad.right_trigger == 0)
        {
            SetPower(-0.3);
        }else{
            SetPidPower(Reeference);
        }

        if(Arm_Gamepad.dpad_up)
        {
            Reeference = 2000;
        }

        if(Arm_Gamepad.dpad_down)
        {
            Reeference = 75;
        }

        LastReference = Reeference;

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
        double power = -PID.returnPower(reference,ArmMotor.MotorCurrentPosition());

        power = addons(power);

        ArmMotor.SetPower(power);
    }

    double addons(double pow)
    {
        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;

        return pow;
    }

}
