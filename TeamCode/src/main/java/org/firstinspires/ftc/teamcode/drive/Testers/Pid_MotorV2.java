package org.firstinspires.ftc.teamcode.drive.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Experminetal;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.MPid_Controller;

@Config
@TeleOp
public class Pid_MotorV2 extends LinearOpMode {

    public DcMotorEx TestMotor;

//    Pid_Controller PD = new Pid_Controller(Kp,Ki,Kd);

    MPid_Controller mpid = new MPid_Controller(Kp,Kd,Ki,max_accel,max_vel);
    Experminetal exp = new Experminetal(Kp,Kd,Ki,max_accel,max_vel);

    private double LastError = 0;
    private double IntegralSum = 0;

    public static double Kp = 0.0085;
    public static double Ki = 0.00002;
    public static double Kd = 0.0;

    public static double max_accel;
    public static double max_vel;

    public double encoder_direction = 1;

    public static int targetPosition = 1000;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet =new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        TestMotor = hardwareMap.get(DcMotorEx.class,"Arm");

        TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TestMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder_direction = GetEncoderDirection(targetPosition);

     //   PD.InitTimer();

        telemetry.addData("Encoder_Direction",encoder_direction);

        double reference = targetPosition * encoder_direction;

        waitForStart();

        while (opModeIsActive()) {

            double state = TestMotor.getCurrentPosition() * encoder_direction;
            double power = encoder_direction * exp.returnPower(targetPosition,state,TestMotor.getVelocity());

            packet.put("encoderDirection", encoder_direction);
            packet.put("power", power);
            packet.put("position", state);
            packet.put("error", LastError);


                TestMotor.setPower(-power);


            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double GetEncoderDirection(int TargetPosition)
    {
        if(TargetPosition < 0){
            return -1;
        }else return 1;
    }

}