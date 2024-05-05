
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*  REV Robot Control Hub Schema
Webcam 1        ->  SN0001
Control Hub:
Motor Port 0    ->  MTR0
Motor Port 1    ->  MTR1
Motor Port 2    ->  MTR2
Motor Port 3    ->  MTR3

Servo Port 0    ->  PWM0
Servo Port 1    ->  PWM1
Servo Port 2    ->  PWM2
Servo Port 3    ->  PWM3
Servo Port 4    ->  PWM4
Servo Port 5    ->  PWM5

Digital Dev 0   ->  DIGDEV0
Digital Dev 1   ->  DIGDEV1
Digital Dev 2   ->  DIGDEV2
Digital Dev 3   ->  DIGDEV3
Digital Dev 4   ->  DIGDEV4
Digital Dev 5   ->  DIGDEV5
Digital Dev 6   ->  DIGDEV6
Digital Dev 7   ->  DIGDEV7

Analog Dev 0    ->  ANDEV0
Analog Dev 1    ->  ANDEV1
Analog Dev 2    ->  ANDEV2
Analog Dev 3    ->  ANDEV3

I2C Bus 0       ->  IMU
I2C Bus 1       ->
I2C Bus 2       ->
I2C Bus 3       ->

Expansion Hub:
Motor Port 0    ->  EXPHUBMTR0
Motor Port 1    ->  EXPHUBMTR1
Motor Port 2    ->  EXPHUBMTR2
Motor Port 3    ->  EXPHUBMTR3

Servo Port 0    ->  EXPHUBPWM0
Servo Port 1    ->  EXPHUBPWM1
Servo Port 2    ->  EXPHUBPWM2
Servo Port 3    ->  EXPHUBPWM3
Servo Port 4    ->  EXPHUBPWM4
Servo Port 5    ->  EXPHUBPWM5

Digital Dev 0   ->  EXPHUBDD0
Digital Dev 1   ->  EXPHUBDD1
Digital Dev 2   ->  EXPHUBDD2
Digital Dev 3   ->  EXPHUBDD3
Digital Dev 4   ->  EXPHUBDD4
Digital Dev 5   ->  EXPHUBDD5
Digital Dev 6   ->  EXPHUBDD6
Digital Dev 7   ->  EXPHUBDD7

Analog Dev 0    ->  EXPHUBANDEV0
Analog Dev 1    ->  EXPHUBANDEV1
Analog Dev 2    ->  EXPHUBANDEV2
Analog Dev 3    ->  EXPHUBANDEV3

I2C Bus 0       ->  COLORSEN
I2C Bus 1       ->  DISTSEN
I2C Bus 2       ->
I2C Bus 3       ->

 */
@Autonomous(name="IterativeOpModeAuto_Test", group="Thomas")
public class IterativeOpModeAuto_Test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopCycleTime = new ElapsedTime();


    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;
    private DcMotor leftRearDriveMotor = null;

    static final double MAX_MOTOR_POW = 0.12;
    static final double MIN_MOTOR_POW = -0.12;


    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    double leftFrontPower  = axial + lateral + yaw;
    double rightFrontPower = axial - lateral - yaw;
    double rightRearPower  = axial + lateral - yaw;
    double leftRearPower   = axial - lateral + yaw;

    double max = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDriveMotor  = hardwareMap.get(DcMotor.class, "EXPHUBMTR0");
        rightFrontDriveMotor  = hardwareMap.get(DcMotor.class, "EXPHUBMTR1");
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "EXPHUBMTR3");
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "EXPHUBMTR2");

        //Test motor direction to confirm the set directions match front of robot
        leftFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        loopCycleTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (runtime.seconds()<5){ //axial forward
            axial = 0.12;
            lateral = 0;
            yaw = 0;
        } else if (runtime.seconds()>5 && runtime.seconds()<10) { //lateral left
            axial = 0;
            lateral = 0.12;
            yaw = 0;
        } else if (runtime.seconds()>10 && runtime.seconds()<15) { //angle up left
            axial = 0.12;
            lateral = 0.12;
            yaw = 0;
        } else if (runtime.seconds()>15 && runtime.seconds()<20) {//yaw clockwise
            axial = 0;
            lateral = 0;
            yaw = 0.12;
        } else {
            axial = 0;
            lateral = 0;
            yaw = 0;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower  = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        rightRearPower  = axial + lateral - yaw;
        leftRearPower   = axial - lateral + yaw;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), 1);
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightRearPower));
        max = Math.max(max, Math.abs(leftRearPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            rightRearPower   /= max;
            leftRearPower  /= max;
        }

        //clip power to max and min constants set from above
        leftFrontPower = Range.clip(leftFrontPower,MIN_MOTOR_POW,MAX_MOTOR_POW);
        rightFrontPower = Range.clip(rightFrontPower,MIN_MOTOR_POW,MAX_MOTOR_POW);
        rightRearPower = Range.clip(rightRearPower,MIN_MOTOR_POW,MAX_MOTOR_POW);
        leftRearPower = Range.clip(leftRearPower,MIN_MOTOR_POW,MAX_MOTOR_POW);

        // Send calculated power to wheels
        leftFrontDriveMotor.setPower(leftFrontPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightRearDriveMotor.setPower(rightRearPower);
        leftRearDriveMotor.setPower(leftRearPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Loop Cycle Time (ms): ", "%4.2f",loopCycleTime.milliseconds());
        telemetry.addData("Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear  Left/Right", "%4.2f, %4.2f", rightRearPower, leftRearPower);
        telemetry.update();

        loopCycleTime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

