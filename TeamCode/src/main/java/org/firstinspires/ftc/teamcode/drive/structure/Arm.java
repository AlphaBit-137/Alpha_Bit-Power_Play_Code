package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pid_Controller;

public class Arm {

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double Reeference = 101;

    Pid_Controller PID = new Pid_Controller(Kp,Ki,Kd);

    public DcMotorEx arm;

    Gamepad Arm_Gamepad;

    public Motor_Skeleton ArmMotor = new Motor_Skeleton(arm);

    public void init(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor.init(ahwMap,"Arm",true);
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

    }

    public void SetPower(double power) {
        ArmMotor.SetPower(power);
    }

    public void SetPidPower(double reference)
    {
        ArmMotor.SetPower(-PID.returnPower(reference,ArmMotor.MotorCurrentPosition()));
    }


}
