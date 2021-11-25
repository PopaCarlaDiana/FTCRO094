
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomiefundatiealbastru", group = "")

public class Autonomiefundatiealbastru extends LinearOpMode {

    DcMotor stanga= null;
    DcMotor dreapta = null;

    DcMotor stangafata = null;
    DcMotor dreaptafata = null;
    Servo carlig =null;

    @Override
    public void runOpMode() throws  InterruptedException {


        ///Motoare
        stanga=hardwareMap.dcMotor.get("stanga");
        dreapta=hardwareMap.dcMotor.get("dreapta");
        stangafata=hardwareMap.dcMotor.get("stangafata");
        dreaptafata=hardwareMap.dcMotor.get("dreaptafata");
        carlig=hardwareMap.servo.get("carlig");


        stanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        dreaptafata.setDirection(DcMotorSimple.Direction.REVERSE);
        dreapta.setDirection(DcMotorSimple.Direction.REVERSE);


        while( !opModeIsActive() && !isStopRequested() )
        {
            telemetry.addData("status", "waiting...");
            telemetry.update();
        }
        DreaptaStanga(0.5,-1400);///dreapta
        FataSpate(0.5, -3000);///spate
        carlig.setPosition(1);
        sleep(2000);
        FataSpate(0.5, 3000);///fata
        carlig.setPosition(0);
        sleep(1000);
        carlig.setPosition(0.5);
        DreaptaStanga(0.5,2450);///stanga
        FataSpate(0.5, -1200);///fata
        DreaptaStanga(0.5,-780);///dreapta
        ///parcare dreapta
        FataSpate(0.5, 900);///spate
        ///parcare stanga
        // FataSpate(0.5, -1300);///fata
        DreaptaStanga(0.5, 2600);///stanga

    }

    public void FataSpate(double power, int distance)
    {
        stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga.setTargetPosition(-distance);
        dreapta.setTargetPosition(-distance);
        stangafata.setTargetPosition(-distance);
        dreaptafata.setTargetPosition(-distance);

        stanga.setPower(power);
        dreapta.setPower(power);
        stangafata.setPower(power);
        dreaptafata.setPower(power);

        while (stanga.isBusy() && stangafata.isBusy() && dreapta.isBusy() && dreaptafata.isBusy() && opModeIsActive())
        {
            telemetry.addData("status", "in fata sau spate");
            telemetry.update();
        }

        stanga.setPower(0);
        dreapta.setPower(0);
        stangafata.setPower(0);
        dreaptafata.setPower(0);

    }
    public void DreaptaStanga(double power, int distance)
    {
        stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga.setTargetPosition(distance);
        dreapta.setTargetPosition(-distance);
        stangafata.setTargetPosition(-distance);
        dreaptafata.setTargetPosition(distance);

        stanga.setPower(power);
        dreapta.setPower(power);
        stangafata.setPower(power);
        dreaptafata.setPower(power);

        while (stanga.isBusy() && stangafata.isBusy() && dreapta.isBusy() && dreaptafata.isBusy() && opModeIsActive())
        {
            telemetry.addData("status", "in dreapta sau stanga");
            telemetry.update();
        }

        stanga.setPower(0);
        dreapta.setPower(0);
        stangafata.setPower(0);
        dreaptafata.setPower(0);

    }
    public void Rotatie(double power, int distance)
    {
        stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga.setTargetPosition(-distance);
        dreapta.setTargetPosition(distance);
        stangafata.setTargetPosition(-distance);
        dreaptafata.setTargetPosition(distance);

        stanga.setPower(power);
        dreapta.setPower(power);
        stangafata.setPower(power);
        dreaptafata.setPower(power);

        while (stanga.isBusy() && stangafata.isBusy() && dreapta.isBusy() && dreaptafata.isBusy() && opModeIsActive())
        {
            telemetry.addData("status", "in dreapta sau stanga");
            telemetry.update();
        }

        stanga.setPower(0);
        dreapta.setPower(0);
        stangafata.setPower(0);
        dreaptafata.setPower(0);

    }
}
