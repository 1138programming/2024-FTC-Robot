package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivebase {
    DcMotor LF,LB,RF,RB;
    IntegratingGyroscope gyro;

    Orientation orientation;
    OpenGLMatrix RotMatrix;
    OpenGLMatrix FieldForwardRot;
    AngularVelocity angle;

    float feildRot;

    public Drivebase(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, IntegratingGyroscope gyro){
        this.LF =LF;
        this.LB =LB;
        this.RB =RB;
        this.RF =RF;
        this.gyro = gyro;

        orientation = gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES);
        RotMatrix =  orientation.getRotationMatrix();

        RotMatrix.get(0,0);
        angle = gyro.getAngularVelocity(AngleUnit.DEGREES);


        orientation.getRotationMatrix().getData();

    }


    public void drive(float x_velocity, float y_velocity, float Rot, boolean reversed, float speed) {
        double max;



        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -y_velocity;  // Note: pushing stick forward gives negative value
        double lateral =  x_velocity;
        double yaw     =  Rot;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (axial + lateral + yaw) * speed;
        double rightFrontPower = (axial - lateral - yaw) * speed;
        double leftBackPower   = (axial - lateral + yaw) * speed;
        double rightBackPower  = (axial + lateral - yaw) * speed;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        if (!reversed) {
            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LB.setPower(leftBackPower);
            RB.setPower(rightBackPower);
        }
        else {
            LF.setPower(-leftFrontPower);
            RF.setPower(-rightFrontPower);
            LB.setPower(-leftBackPower);
            RB.setPower(-rightBackPower);
        }


    }

    public void driveFeildRelative(float x_velocity, float y_velocity, float Rot, boolean reversed, float speed) {
        float nx_velocity;
        float ny_velocity;

        VectorF nMatrix = new VectorF(new float[] {x_velocity, y_velocity});

        orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
//        RotMatrix =  orientation.getRotationMatrix();



        RotMatrix = Orientation.getRotationMatrix(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,  getAdjustedAngle() - feildRot, (float) 0, (float) 0);

//        angle = gyro.getAngularVelocity(AngleUnit.DEGREES);



        VectorF SVector = new VectorF(new float[] {x_velocity, y_velocity, 0, 1});

        VectorF nVector =  RotMatrix.multiplied(SVector);

        drive(nVector.get(0),nVector.get(1),Rot,reversed, speed);













    }


    public float getAdjustedAngle() {
        orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        if (orientation.firstAngle < 0  ) {
            return 180 + (180 + orientation.firstAngle);
        }
        else {
            return orientation.firstAngle;
        }
    }

    public void resetfeildrot() {


        feildRot = getAdjustedAngle();

    }

    public void driveDist() {

    }



}
