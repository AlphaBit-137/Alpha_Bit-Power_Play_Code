package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Skeleton {

    DcMotor ThisMotor;
    STATES MotorState = STATES.STOP;
    HardwareMap hwMap = null;

   public Motor_Skeleton(DcMotor ThisMotor){
        this.ThisMotor = ThisMotor;
    }

    public void init(HardwareMap ahwMap,String MotorName,boolean IsReversed) {
        hwMap = ahwMap;
        ThisMotor = hwMap.get(DcMotor.class, MotorName);
        ThisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ThisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IsReversed(IsReversed);
        ThisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ThisMotor.setPower(0);
    }

    enum STATES{

        UP{
            STATES GoTo(STATES goining_state){
                return goining_state;
            }
        },
        DOWN{
            STATES GoTo(STATES goining_state){
                return goining_state;
            }
        },
        STOP{
            STATES GoTo(STATES goining_state){
                return goining_state;
            }
        };

        abstract STATES GoTo(STATES going_state);

    }

    public void StateUpdate(double powerDOWN, double powerUP){
        switch (MotorState){
            case UP:
                ThisMotor.setPower(powerUP);
                break;
            case DOWN:
                ThisMotor.setPower(powerDOWN);
                break;
            case STOP:
                ThisMotor.setPower(0);
                break;
        }
    }

    public void GoTo(String State_String){
        switch(State_String) {
            case "up":
                MotorState = STATES.UP;
                break;
            case "down":
                MotorState = STATES.DOWN;
                break;
            case "stop":
                MotorState = STATES.STOP;
                break;
        }
    }

    public void IsReversed(boolean isreversed)
    {
        if(isreversed){
            ThisMotor.setDirection(DcMotor.Direction.REVERSE);
        }else ThisMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }



}
