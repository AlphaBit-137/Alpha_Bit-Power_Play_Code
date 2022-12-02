package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

/*
*
*Experimental class used to calculate the power returned to the motor using PID
*
 */

@Config
public class Pid_Motor extends LinearOpMode {

    public DcMotor TestMotor;

    ElapsedTime timer = new ElapsedTime();

   private double LastError = 0;
   private double IntegralSum = 0;

   public static double Kp = 0.0;
   public static double Ki = 0.0;
   public static double Kd = 0.0;

   public int targetPosition = 100;


   private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TelemetryPacket packet =new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        TestMotor = hardwareMap.get(DcMotor.class,"SliderPID");

        TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
           double power = returnPower(targetPosition,TestMotor.getCurrentPosition());

           packet.put("power",power);
           packet.put("position",TestMotor.getCurrentPosition());
           packet.put("error",LastError);


           TestMotor.setPower(power);

          dashboard.sendTelemetryPacket(packet);
        }
        }

        public double returnPower(double reference, double state){
        double error = reference - state;

        IntegralSum += error * timer.seconds();

        double derivative = (error - LastError) / timer.seconds();

        LastError = error;

        double outpput = (error * Kp) + (derivative * Kd) + (IntegralSum * Ki);

        return outpput;
        }

    }

