package org.firstinspires.ftc.teamcode.drive.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
*
*Experimental class used to calculate the power returned to the motor using PID
*
 */

@Config
@TeleOp
public class Pid_Motor extends LinearOpMode {

    public DcMotor TestMotor;

    ElapsedTime timer = new ElapsedTime();

   private double LastError = 0;
   private double IntegralSum = 0;

   public static double Kp = 0.0085;
   public static double Ki = 0.00002;
   public static double Kd = 0.0;

   public double encoder_direction = 1;

   public static int targetPosition = 1000;


   private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet =new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        TestMotor = hardwareMap.get(DcMotor.class,"Slider");

        TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TestMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder_direction = GetEncoderDirection(targetPosition);

        telemetry.addData("Encoder_Direction",encoder_direction);

        double reference = targetPosition * encoder_direction;

        waitForStart();

        while (opModeIsActive()) {

            double state = TestMotor.getCurrentPosition() * encoder_direction;
            double power = encoder_direction * returnPower(reference, state);

            packet.put("encoderDirection", encoder_direction);
            packet.put("power", power);
            packet.put("position", state);
            packet.put("error", LastError);

            if (TestMotor.getCurrentPosition() >= 0 && state != reference) {
                TestMotor.setPower(power);
            } else
            {
                reference = 500;
                sleep(10000);
            }

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

        public double GetEncoderDirection(int TargetPosition)
        {
            if(TargetPosition < 0){
                return -1;
            }else return 1;
        }

    }

