package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

    @Autonomous(name = "Tacauto")

    public class Tacauto extends Taco_Super_class {


        double Shooter_Speed;
        double Rounds;
        double Zone;

        @Override
        public void runOpMode() {
            List<Recognition> recognitions;
            double index;
            Right_Front = hardwareMap.get(DcMotor.class, "Right_Front");
            Right_Rear = hardwareMap.get(DcMotor.class, "Right_Rear");
            Left_Front = hardwareMap.get(DcMotor.class, "Left_Front");
            Left_Rear = hardwareMap.get(DcMotor.class, "Left_Rear");
            arm = hardwareMap.get(DcMotor.class, "arm");
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            wheels = hardwareMap.get(DcMotor.class, "wheels");
            indexer = hardwareMap.get(Servo.class, "indexer");
            pincher = hardwareMap.get(Servo.class, "pincher");

            initialization(true);

            androidSoundPool = new AndroidSoundPool();
            vuforiaUltimateGoal = new VuforiaCurrentGame();
            tfodUltimateGoal = new TfodCurrentGame();

            telemetry.addData(">", "Wait To Start!!!!!!!!!!!!!!!!!!");
            telemetry.update();
            androidSoundPool.initialize(SoundPlayer.getInstance());
            ((DcMotorEx) Right_Front).setTargetPositionTolerance(10);
            ((DcMotorEx) Right_Rear).setTargetPositionTolerance(10);
            ((DcMotorEx) Left_Front).setTargetPositionTolerance(10);
            ((DcMotorEx) Left_Rear).setTargetPositionTolerance(10);
            Rounds = 0;
            Zone = 1;
            Shooter_Speed = 0.65;
            // Put initialization blocks here.
            Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            wheels.setDirection(DcMotorSimple.Direction.REVERSE);
            // You will have to determine which motor to reverse for your robot.
            // In this example, the right motor was reversed so that positive
            // applied power makes it move the robot in the forward direction.
            //Left_Front.setDirection(DcMotorSimple.Direction.REVERSE);
            // You will have to determine which motor to reverse for your robot.
            // In this example, the right motor was reversed so that positive
            // applied power makes it move the robot in the forward direction.
            //Left_Rear.setDirection(DcMotorSimple.Direction.REVERSE);
            pincher.setPosition(0.99);
            vuforiaUltimateGoal.initialize(
                    "", // vuforiaLicenseKey
                    hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                    "", // webcamCalibrationFilename
                    false, // useExtendedTracking
                    true, // enableCameraMonitoring
                    VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                    0, // dx
                    0, // dy
                    0, // dz
                    0, // xAngle
                    0, // yAngle
                    0, // zAngle
                    true); // useCompetitionFieldTargetLocations
            // Set min confidence threshold to 0.7
            tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
            // Initialize TFOD before waitForStart.
            // Init TFOD here so the object detection labels are visible
            // in the Camera Stream preview window on the Driver Station.
            tfodUltimateGoal.activate();
            // Enable following block to zoom in on target.
            tfodUltimateGoal.setZoom(2.5, 16 / 9);
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            androidSoundPool.play("RawRes:ss_roger_roger");
            waitForStart();
            if (opModeIsActive()) {
                // Get a list of recognitions from TFOD.
                recognitions = tfodUltimateGoal.getRecognitions();
                while (Rounds < 16) {
                    // If list is empty, inform the user. Otherwise, go
                    // through list and display info for each recognition.
                    if (recognitions.size() == 0) {
                        Zone = 1;
                        telemetry.addData("Zone", Zone);
                        Rounds += 1;
                        telemetry.addData("Zone", Zone);
                        telemetry.addData("Rounds", Rounds);
                        telemetry.update();
                    } else {
                        index = 0;
                        // Iterate through list and call a function to
                        // display info for each recognized object.
                        for (Recognition recognition_item : recognitions) {
                            recognition = recognition_item;
                            // Display info.
                            displayInfo(index);
                            // Increment index.
                            index = index + 1;
                            Object2();  
                            telemetry.addData("Zone", Zone);
                            telemetry.addData("Zone", Zone);
                            telemetry.addData("Rounds", Rounds);
                            telemetry.update();
                        }
                    }
                }
                Stop_And_Reset_Encoders();
                while (nextstep(500,0,0)){
                    teledrive(1*foregoal(500),1*rightgoal(0),1*turngoal(0));
                    drivetele();
                    shoot(true,false);
                }
                teledrive(0,0,0);
                /*Right_Front.setTargetPosition(500);
                Right_Rear.setTargetPosition(500);
                Left_Front.setTargetPosition(520);
                Left_Rear.setTargetPosition(520);*/

                Stop_And_Reset_Encoders();
            }
            Stop_And_Reset_Encoders();
            while (nextstep(500,0,0)){
                teledrive(foregoal(500),rightgoal(0),turngoal(0));
                drivetele();
            }
            sleep(1000);
            if (Zone == 1) {
                Zone_A();
            } else if (Zone == 2) {
                Zone_B();
            } else if (Zone == 3) {
                Zone_C();
            }

            androidSoundPool.close();
            vuforiaUltimateGoal.close();
            tfodUltimateGoal.close();
        }

        public void Zone_A() {
            Stop_And_Reset_Encoders();
            //driveright(1050,.8);
            sleep(1000);
            while(nextstep(0,1050,0)){
                teledrive(foregoal(0),rightgoal(1050),turngoal(0));
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            //driveForwardEncoders(1150,.8);
            while (nextstep(1250,0,0)){
                teledrive(foregoal(1250),rightgoal(0),turngoal(0));
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            sleep(1500);
            Shooter_Speed = 0.67;
            sleep(10);
            Shoot2();
            Shoot2();
            sleep(500);
            //driveForwardEncoders(900,.8);
            while (nextstep(900,0,0)){
                teledrive(foregoal(900),rightgoal(0),turngoal(0));
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            sleep(1000);
            //driveleft(800,.8);
            while (nextstep(0,-800,0)){
                teledrive(foregoal(0),rightgoal(-800),turngoal(0));
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            sleep(1000);
            Right_Front.setTargetPosition(-2000);
            Right_Rear.setTargetPosition(-2000);
            Left_Front.setTargetPosition(2000);
            Left_Rear.setTargetPosition(2000);
            Run_To_Position();
            Motor_Power();

            while (nextstep(0,0,2000)){
                teledrive(foregoal(0),rightgoal(0),turngoal(2000));;
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            sleep(2000);
            driveright(800,.8);
            while (nextstep(0,800,0)){
                teledrive(foregoal(00),rightgoal(800),turngoal(0));
            }
            teledrive(0,0,0);
            Stop_And_Reset_Encoders();

            sleep(2000);
            arm.setPower(-1);
            sleep(1000);
            arm.setPower(0);
            pincher.setPosition(0);
            sleep(250);
            arm.setPower(0.5);
            sleep(600);
            arm.setPower(0);
            sleep(250);
            arm.setPower(0.5);
            sleep(500);
            arm.setPower(0);
        }

        public void Zone_B() {
            Stop_And_Reset_Encoders();
            driveright(1050,.8);
            sleep(1000);
            Stop_And_Reset_Encoders();
            Right_Front.setTargetPosition(1225);
            Right_Rear.setTargetPosition(1225);
            Left_Front.setTargetPosition(1150);
            Left_Rear.setTargetPosition(1150);
            Run_To_Position();
            Motor_Power();
            sleep(1500);
            Stop_And_Reset_Encoders();
            Shooter_Speed = 0.67;
            Shoot2();
            Shoot2();
            Stop_And_Reset_Encoders();
            Run_To_Position();
            Motor_Power();
            sleep(1000);
            arm.setPower(-1);
            sleep(1000);
            arm.setPower(0);
            pincher.setPosition(0);
            sleep(250);
            arm.setPower(0.5);
            sleep(600);
            arm.setPower(0);
            sleep(250);
            Stop_And_Reset_Encoders();
            driveBackwardEncoders(700,.8);
            sleep(1000);
            driveleft(800,.8);
            wheels.setPower(1);
            sleep(1000);
            driveBackwardEncoders(1600,.33);
            sleep(1500);
            shooter.setPower(0.6);
            sleep(1000);
            Stop_And_Reset_Encoders();
            Shooter_Speed = 0.6;
            Shoot2();
            shooter.setPower(Shooter_Speed);
            Shoot2();
            shooter.setPower(Shooter_Speed);
            Stop_And_Reset_Encoders();
            driveForwardEncoders(700,.8);
            Run_To_Position();
            Motor_Power();
            sleep(1000);
            Right_Front.setTargetPosition(-200);
            Right_Rear.setTargetPosition(-2000);
            Left_Front.setTargetPosition(2000);
            Left_Rear.setTargetPosition(2000);
            Run_To_Position();
            Motor_Power();
            sleep(1000);
            Stop_And_Reset_Encoders();
            arm.setPower(0.5);
            sleep(500);
            arm.setPower(0);
        }

        public void Zone_C() {
            driveright(1050,.8);
            sleep(1000);
            Stop_And_Reset_Encoders();
            Right_Front.setTargetPosition(1225);
            Right_Rear.setTargetPosition(1225);
            Left_Front.setTargetPosition(1150);
            Left_Rear.setTargetPosition(1150);
            Run_To_Position();
            Motor_Power();
            sleep(1000);
            Shooter_Speed = 0.67;
            Shoot2();
            Shoot2();
            sleep(500);
            Stop_And_Reset_Encoders();
            driveForwardEncoders(2800,.8);
            sleep(3000);
            Stop_And_Reset_Encoders();
            driveleft(100,.8);
            sleep(1000);
            Right_Front.setTargetPosition(-2000);
            Right_Rear.setTargetPosition(-2000);
            Left_Front.setTargetPosition(2000);
            Left_Rear.setTargetPosition(2000);
            Run_To_Position();
            Motor_Power();
            sleep(2000);
            Stop_And_Reset_Encoders();
            driveleft(1500,.8);
            sleep(2000);
            driveBackwardEncoders(300,.8);
            sleep(1000);
            Stop_And_Reset_Encoders();
            arm.setPower(-1);
            sleep(1000);
            arm.setPower(0);
            pincher.setPosition(0);
            sleep(250);
            arm.setPower(0.5);
            sleep(600);
            arm.setPower(0);
            sleep(250);
            arm.setPower(0.5);
            sleep(500);
            arm.setPower(0);
            Stop_And_Reset_Encoders();
            Right_Front.setTargetPosition(0);
            Right_Rear.setTargetPosition(0);
            Left_Front.setTargetPosition(600);
            Left_Rear.setTargetPosition(600);
            Run_To_Position();
            Motor_Power();
            sleep(1000);
            Stop_And_Reset_Encoders();
            Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Front.setPower(1);
            Right_Rear.setPower(1);
            Left_Rear.setPower(1);
            Left_Front.setPower(1);
            sleep(800);
            DriveOff();
            sleep(30000);
        }

    }
