package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

public abstract class Taco_Super_class extends LinearOpMode {

    public AndroidSoundPool androidSoundPool;
    public DcMotor Right_Front;
    public DcMotor Right_Rear;
    public DcMotor Left_Front;
    public DcMotor Left_Rear;
    public DcMotor arm;
    public DcMotor shooter;
    public DcMotor wheels;

    public Servo indexer;
    public Servo pincher;

    public static double kP = 0.05;
    public static double kI = 0.0;                              // these will be used in the PID methods
    public static double kD = 0.0;

    public VuforiaCurrentGame vuforiaUltimateGoal;
    public TfodCurrentGame tfodUltimateGoal;

    public BNO055IMU imu;
    public float imuStartingPosition;

    double frontlefttarget = 0;
    double frontrighttarget = 0;
    double backrighttarget = 0;
    double backlefttarget = 0;
    double shootertarget = 0;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    double Shooter_Speed;
    double Rounds;
    double Zone;
    Recognition recognition;
    public ElapsedTime tacotimer = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();

    double tacotimers;
    double shooterplace;
    double shooterdif;

    double dfore;
    double dright;
    double dturn;

    public void initialization(boolean autonomous) {
        initarm();
        initindexer();
        initwheels();
        initpincher();

        if (autonomous) {
            telemetry.addLine("imu init");
            telemetry.update();
            initiate_imu();

            telemetry.addLine("drive Init");
            telemetry.update();
            //initDrive


            Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else {
            Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }
        telemetry.addLine("Finished init - GO PRAISE JESUS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        telemetry.update();
    }

    public void initiate_imu() {

        telemetry.addLine("Initiating IMU");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (opModeIsActive()) {

            imuStartingPosition = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        }
    }


    // --------------------------------Motors--------------------------------------
    public void initarm() {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*public void initshooter (double distance, double power){
        shooter = hardwareMap.dcMotor.get("Shooter");

        IMUstraightDouble(turn);

       while (shooter.getCurrentPosition() < shootertarget && opModeIsActive()){

           double turn = IMUstraightDouble(turn);
           powerDriveTrain((power + turn), (power - turn));
           telemetry.update();
       }
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //pidafy thathathathat meth
    }*/

    public void initwheels() {
        wheels = hardwareMap.dcMotor.get("wheels");
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //--------------------------------------Servos-----------------------------------
    //public void initarm_servo(){arm_servo = hardwareMap.get(Servo.class, "arm_servo");}

    public void initindexer() {
        indexer = hardwareMap.get(Servo.class, "indexer");
    }

    public void initpincher() {
        pincher = hardwareMap.get(Servo.class, "pincher");
    }

    //----------------------------Drive Methods---------------------------------------------
    public void DriveOff() {
        Left_Rear.setPower(0);
        Right_Rear.setPower(0);
        Left_Front.setPower(0);
        Right_Front.setPower(0);
    }

    public void driveForwardEncoders(double distance, double power) {
        backlefttarget = (Left_Rear.getCurrentPosition() + distance);
        backrighttarget = (Right_Rear.getCurrentPosition() + distance);
        frontlefttarget = (Left_Front.getCurrentPosition() + distance);
        frontrighttarget = (Right_Front.getCurrentPosition() + distance);

        while (Left_Rear.getCurrentPosition() < backlefttarget
                /*&& leftFront.getCurrentPosition() < frontlefttarget
                && rightBack.getCurrentPosition() < backrighttarget
                && rightFront.getCurrentPosition() < frontrighttarget*/ && opModeIsActive()) {


            Left_Rear.setPower(power);
            Left_Front.setPower(power);
            Right_Rear.setPower(power);
            Right_Front.setPower(power);

            telemetry.addData("Left_Rear", Left_Rear.getCurrentPosition());
            telemetry.addData("Left_Front", Left_Front.getCurrentPosition());
            telemetry.addData("Right_Rear", Right_Rear.getCurrentPosition());
            telemetry.addData("Right_Front", Right_Front.getCurrentPosition());

            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }


    public void driveBackwardEncoders(double distance, double power) {

        backlefttarget = (Left_Rear.getCurrentPosition() - distance);
        backrighttarget = (Right_Rear.getCurrentPosition() - distance);
        frontlefttarget = (Left_Front.getCurrentPosition() - distance);
        frontrighttarget = (Right_Front.getCurrentPosition() - distance);

        while (Left_Rear.getCurrentPosition() > backlefttarget
               /* && leftFront.getCurrentPosition() > frontlefttarget
                && rightBack.getCurrentPosition() > backrighttarget
                && rightFront.getCurrentPosition() > frontrighttarget*/ && opModeIsActive()) {
            Left_Rear.setPower(-power);
            Left_Front.setPower(-power);
            Right_Rear.setPower(-power);
            Right_Front.setPower(-power);

            telemetry.addData("Left_Rear", Left_Rear.getCurrentPosition());
            telemetry.addData("Left_Front", Left_Front.getCurrentPosition());
            telemetry.addData("Right_Rear", Right_Rear.getCurrentPosition());
            telemetry.addData("Right_Front", Right_Front.getCurrentPosition());
            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void driveleft(double distance, double power) {
        Left_Front.setPower(-power);
        Left_Rear.setPower(power);
        Right_Front.setPower(power);
        Right_Rear.setPower(-power);

        backlefttarget = (Left_Rear.getCurrentPosition() + distance);
        backrighttarget = (Right_Rear.getCurrentPosition() - distance);
        frontlefttarget = (Left_Front.getCurrentPosition() - distance);
        frontrighttarget = (Right_Front.getCurrentPosition() + distance);

        while (/*leftFront.getCurrentPosition() > frontlefttarget
               &&*/ Left_Rear.getCurrentPosition() < backlefttarget
               /* && rightFront.getCurrentPosition() < frontrighttarget
                && rightBack.getCurrentPosition() > backrighttarget*/ && opModeIsActive()) {
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void driveright(double distance, double power) {
        Left_Front.setPower(power);
        Left_Rear.setPower(-power);
        Right_Front.setPower(-power);
        Right_Rear.setPower(power);

        backlefttarget = (Left_Rear.getCurrentPosition() - distance);
        backrighttarget = (Right_Rear.getCurrentPosition() + distance);
        frontlefttarget = (Left_Front.getCurrentPosition() + distance);
        frontrighttarget = (Right_Front.getCurrentPosition() - distance);

        while (//leftFront.getCurrentPosition() < frontlefttarget
            /*&&*/ Left_Rear.getCurrentPosition() > backlefttarget
                /* && rightFront.getCurrentPosition() > frontrighttarget
                 *//*&&*//* rightBack.getCurrentPosition() < backrighttarget*/ && opModeIsActive()) {

        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void rotateLeft(double targetAngle, double power) {
        Left_Rear.setPower(-power);
        Left_Front.setPower(-power);
        Right_Rear.setPower(power);
        Right_Front.setPower(power);

        while (opModeIsActive() && getAngle() < targetAngle) {
            telemetry.addData("curentAngle", getAngle());
            telemetry.addData("targetAngle", targetAngle);
            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();

    }

    public void rotateRight(double targetAngle, double power) {
        Left_Rear.setPower(power);
        Left_Front.setPower(power);
        Right_Rear.setPower(-power);
        Right_Front.setPower(-power);

        while (opModeIsActive() && getAngle() > targetAngle) {
            telemetry.addData("curentAngle", getAngle());
            telemetry.addData("targetAngle", targetAngle);
            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void Stop_And_Reset_Encoders() {
        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        Right_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    public void Run_To_Position() {
        Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Motor_Power() {
        Right_Front.setPower(0.8);
        Right_Rear.setPower(0.8);
        Left_Front.setPower(0.8);
        Left_Rear.setPower(0.8);
    }

    //--------------------------------- imu stuff i think----------------------------


    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    //______________________pid stuff__________________________________________________________

    PIDController pid = new PIDController(new PIDCoefficients(kP, kI, kD));


    public double IMUstraightDouble(double targetAngle) {

        double currentAngle = getAngle();
        pid.setCoeffs(new PIDCoefficients(kP, kI, kD));
        return pid.update(currentAngle - targetAngle);
    }

    public double distancePIDDouble(double power) {

        //double shooter = getAngle();
        pid.setCoeffs(new PIDCoefficients(kP, kI, kD));
        return pid.update(-shootertarget);
    }


    // -------------------------other stuff--------------------------------------------------
    public void Motor_Power_33() {
        Right_Front.setPower(0.33);
        Right_Rear.setPower(0.33);
        Left_Front.setPower(0.33);
        Left_Rear.setPower(0.33);
    }

    public void displayInfo(double i) {
        if (opModeIsActive()) {
            // Display label info.
            telemetry.addData("Zone", Zone);
            telemetry.update();
        }
    }

    public void Object2() {
        if (opModeIsActive()) {
            telemetry.addData("Here", "3");
            Rounds += 1;
            if (recognition.getLabel().equals("Single")) {
                Zone = 2;
            } else if (recognition.getLabel().equals("Quad")) {
                Zone = 3;
            }
        }
    }

    public void Motor_Power_75() {
        Right_Front.setPower(0.75);
        Right_Rear.setPower(0.75);
        Left_Front.setPower(0.75);
        Left_Rear.setPower(0.75);
    }

    private void Shoot1() {
        shooter.setPower(Shooter_Speed);
        sleep(2000);
        shooter.setPower(Shooter_Speed);
        indexer.setPosition(0.3);
        shooter.setPower(Shooter_Speed);
        sleep(1000);
        shooter.setPower(Shooter_Speed);
        indexer.setPosition(0.51);
    }

    public void Shoot2() {
        shoot(false,true);
        sleep(500);
        shoot(false,true);
        indexer.setPosition(0);
        shoot(false,true);
        sleep(750);
        shoot(false,true);
        indexer.setPosition(0.65);
        shoot(false,true);
        sleep(750);
        shoot(false,true);
        indexer.setPosition(0);
        shoot(false,true);
        sleep(750);
        shoot(false,true);
        indexer.setPosition(0.65);
        shoot(false,false);
    }

    public void powerDriveTrain(double leftPower, double rightPower) {
        double max = 1.0;
        max = Math.max(max, Math.abs(leftPower));
        max = Math.max(max, Math.abs(rightPower));
        leftPower /= max;
        rightPower /= max;

        Left_Rear.setPower(leftPower);
        Left_Front.setPower(leftPower);
        Right_Rear.setPower(rightPower);
        Right_Front.setPower(rightPower);

    }

    public double shoot(boolean powershots, boolean normalshots){

        if(powershots){
            shooter.setPower(7.5*(1/(shooterdif)));
        }
        else if(normalshots){
            shooter.setPower(9.3*(1/(shooterdif)));
        }
        else{
            shooter.setPower(0);
        }
        if((tacotimer.seconds()-tacotimers)>.01) {

            shooterdif = shooter.getCurrentPosition() - shooterplace;

            shooterplace = shooter.getCurrentPosition();

            tacotimers = tacotimer.seconds();
        }
        return shooterdif;
    }

    public int rect(){
        return(-Left_Front.getCurrentPosition()/4 +Left_Rear.getCurrentPosition()/4 - Right_Front.getCurrentPosition()/4 + Right_Rear.getCurrentPosition()/4);
    }
    public int fect(){
        return(-Left_Front.getCurrentPosition()/4 - Left_Rear.getCurrentPosition()/4 + Right_Front.getCurrentPosition()/4 + Right_Rear.getCurrentPosition()/4);
    }
    public int tect(){
        return(-Left_Front.getCurrentPosition()/4 - Left_Rear.getCurrentPosition()/4 - Right_Front.getCurrentPosition()/4 - Right_Rear.getCurrentPosition()/4);
    }

    public void teledrive(double forward,double right,double turnc){
        Left_Rear.setPower(forward + right - turnc);
        Left_Front.setPower(forward - right - turnc);
        Right_Rear.setPower(-forward + right - turnc);
        Right_Front.setPower(-forward - right - turnc);

    }

    /*public void goal(double fore,double right,double turn){


        dfore = (fore-fect())*.2;
        dright = (right-rect())*.3;
        dturn = (turn-tect())*.1;
        telemetry.addData("dfore",dfore);
        telemetry.addData("dright",dright);
        telemetry.addData("dturn",dturn);
        telemetry.update();

        teledrive(dfore,dright,dturn);
        Left_Rear.setPower(dfore + dright - dturn);
        Left_Front.setPower(dfore - dright - dturn);
        Right_Rear.setPower(-dfore + dright - dturn);
        Right_Front.setPower(-dfore - dright - dturn);
    }*/
    public double foregoal(double goal){
        telemetry.addData("forewards delta",(goal-fect()));
        return (goal+fect())*.0009;

    }
    public double rightgoal(double goal){
        telemetry.addData("rightwards delta",(goal-rect()));
        return (goal-rect())*.001;
    }
    public double turngoal(double goal){
        telemetry.addData("turnwards delta",(goal-tect()));
        return (goal+tect())*.001;
    }
    public boolean nextstep(double forego, double rightgo, double turngo){
        if (((Math.abs(rect()-rightgo)<20) && (Math.abs(fect()-forego)<20) && (Math.abs(tect()-turngo)<20))){
            timer.reset();
        }
        if (timer.seconds()>.2 && ((Math.abs(rect()-rightgo)<20) && (Math.abs(fect()-forego)<20) && (Math.abs(tect()-turngo)<20))){
            return false;
        }
        else{
            return true;
        }
    }
    public void drivetele(){
        telemetry.addData("forward encoders",fect());
        telemetry.addData("right encoders",rect());
        telemetry.addData("turn encoders",tect());
        telemetry.update();

    }
}
