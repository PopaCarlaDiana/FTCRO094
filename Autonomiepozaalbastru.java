package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import java.util.Timer;

@Autonomous(name="Autonomiepozaalbastru", group = "")

public class Autonomiepozaalbastru extends LinearOpMode {

    DcMotor stanga= null;
    DcMotor dreapta = null;

    DcMotor stangafata = null;
    DcMotor dreaptafata = null;
    Servo carlig =null;


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true ;

    private static final String VUFORIA_KEY =
            "AcqnxRj/////AAABmb40oUHsVkK5idaHa7HPA0sIgYQdZ3iK5XezIxbnb/AupAbgEOVvFZPoeN2aRe7g/MaeMq4Xcbk9tiQGXz8ncrwBBwUwPT+NIRPfcQQQBzS/R/bT0gLK5AVZ+S5xgCUXiPrd1obzxcLeZGVHfGCJ/mt3heDIB3R7oDAbFHf3Hgt+MSJ+fw3sPTHuEYquye0TxKe3lH54+5Xn1k1Xvyd8mGl4RiLNLD4gVloYmeRUzSWr8bPXzM7QQw0xAmVVRllH2cKNSuFmztwCr5CpIBap5ijAt5U6dnYu61GFadUA5XUpY+4u1+uPDSquFeLFGqN6iAJDtBE0d136riH53V/fJrXgDm3XusbOea9J+lgmErTe";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;


    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


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

        ///vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.33f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.69f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 1.65f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);


        ///////////////////////////////////////////////////////////
        CameraDevice.getInstance().setFlashTorchMode(true);
        targetsSkyStone.activate();
        FataSpate(0.7, -2200);///spate

        targetVisible = false;
        long i=0;
        ///primul cub
        while(targetVisible == false&&i<50000) {
            ///targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                targetVisible = true;
                telemetry.addData("Visible Target","visible");
            }
            telemetry.update();
            // Provide feedback as to where the robot is located (if we know).
            if (!targetVisible) {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            i++;
        }

        int k=1;
        /// primul cub
        if(targetVisible==true) {
            FataSpate(0.7, -650);///fata
            DreaptaStanga(0.5,380);///dreapta
            carlig.setPosition(1);
            sleep(1500);
            FataSpate(0.5, -250);///fata
            FataSpate(0.5, 900);///spate
            DreaptaStanga(0.5,3800);///dreapta
            carlig.setPosition(0);
            sleep(500);
            carlig.setPosition(0.5);
            FataSpate(0.5, -350);
            DreaptaStanga(0.5, -1300);///stanga
        }
        else{//1 - merge
            ///al doilea cub
            k++;
            DreaptaStanga(0.5,-730);///stanga - trecerea la urmatorul cub
            sleep(1000);///analiza cubului
            i=0;
            while(targetVisible == false&&i<50000){
                ///targetVisible = false;
                if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                    targetVisible = true;
                    telemetry.addData("Visible Target","visible");
                    ///telemetry.addData("vasile","vasile");
                }
                telemetry.update();
                // Provide feedback as to where the robot is located (if we know).
                if (!targetVisible) {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
                i++;
            }
            ///al doilea cub
            if(targetVisible==true) {
                FataSpate(0.7, -650);///spate
                DreaptaStanga(0.5,360);///dreapta
                carlig.setPosition(1);
                sleep(1500);
                FataSpate(0.5, -250);///spate
                FataSpate(0.5, 900);///fata
                DreaptaStanga(0.5,4400);///dreapta
                carlig.setPosition(0);
                sleep(500);
                carlig.setPosition(0.5);
                FataSpate(0.5, -400);
                DreaptaStanga(0.5, -1220);///stanga
            }
            else//2 - merge
            {
                ///al treilea cub
                k++;
                DreaptaStanga(0.5,-750);///stanga
                sleep(1000);
                i=0;
                while(targetVisible == false&&i<50000) {
                    ///targetVisible = false;
                    if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                        targetVisible = true;
                        telemetry.addData("Visible Target","visible");

                    }
                    telemetry.update();
                    // Provide feedback as to where the robot is located (if we know).
                    if (!targetVisible) {
                        telemetry.addData("Visible Target", "none");
                    }
                    telemetry.update();
                    i++;
                }
                if(targetVisible==true) {
                    FataSpate(0.7, -650);///spate
                    DreaptaStanga(0.5,380);///dreapta
                    carlig.setPosition(1);
                    sleep(1500);
                    FataSpate(0.5, -250);///spate
                    FataSpate(0.5, 900);///fata
                    DreaptaStanga(0.5,5350);///dreapta
                    carlig.setPosition(0);
                    sleep(500);
                    carlig.setPosition(0.5);
                    FataSpate(0.5, -400);
                    DreaptaStanga(0.5, -1220);///stanga
                }
                else{//3 - merge
                    ///al patrulea cub
                    k++;
                    DreaptaStanga(0.5,-750);///stanga
                    sleep(1000);
                    i=0;
                    while(targetVisible == false&&i<50000) {
                        ///targetVisible = false;
                        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                            targetVisible = true;
                            telemetry.addData("Visible Target","visible");

                        }
                        telemetry.update();
                        // Provide feedback as to where the robot is located (if we know).
                        if (!targetVisible) {
                            telemetry.addData("Visible Target", "none");
                        }
                        telemetry.update();
                        i++;
                    }
                    if(targetVisible==true) {
                        FataSpate(0.7, -650);///spate
                        DreaptaStanga(0.5,380);///dreapta
                        carlig.setPosition(1);
                        sleep(1500);
                        FataSpate(0.5, -250);///spate
                        FataSpate(0.5, 900);///fata
                        DreaptaStanga(0.5,6100);///dreapta
                        carlig.setPosition(0);
                        sleep(500);
                        carlig.setPosition(0.5);
                        FataSpate(0.5, -400);
                        DreaptaStanga(0.5, -1220);///stanga
                    }
                    else{//4
                        ///al cincilea cub
                        k++;
                        DreaptaStanga(0.5,-750);///stanga
                        sleep(1000);
                        i=0;
                        while(targetVisible == false&&i<50000) {
                            ///targetVisible = false;
                            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                                targetVisible = true;
                                telemetry.addData("Visible Target","visible");

                            }
                            telemetry.update();
                            // Provide feedback as to where the robot is located (if we know).
                            if (!targetVisible) {
                                telemetry.addData("Visible Target", "none");
                            }
                            telemetry.update();
                            i++;
                        }
                        if(targetVisible==true) {
                            FataSpate(0.7, -650);///spate
                            DreaptaStanga(0.5,380);///dreapta
                            carlig.setPosition(1);
                            sleep(1500);
                            FataSpate(0.5, -250);///spate
                            FataSpate(0.5, 900);///fata
                            DreaptaStanga(0.5,6800);///dreapta
                            carlig.setPosition(0);
                            sleep(500);
                            carlig.setPosition(0.5);
                            FataSpate(0.5, -400);
                            DreaptaStanga(0.5, -1220);///stanga
                        }
                        else{//5
                            ///al saselea cub
                            ///pleaca fara cub
                        }
                    }
                }
            }
        }
        ////daca merge in interior
        /// FataSpate(0.7, 440);///spate
        ///daca merge in exterior


        ////////////////////////////////////////////////////

        targetsSkyStone.deactivate();

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
