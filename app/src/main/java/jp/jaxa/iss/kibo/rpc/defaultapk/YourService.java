package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point3;

import java.lang.annotation.Target;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private static camera_params camparams;
    private static Mat camMatrix;
    private static Mat dstMatrix;

    private static zbarQR zbarQR_obj;

    public static void setCamCalibration() {
        camparams = new camera_params();
        camparams.process_camera();

        camMatrix=camparams.getCamMatrix();
        dstMatrix=camparams.getDistortionMat();
    }

    @Override
    protected void runPlan1(){
        api.startMission();
        setCamCalibration();

        // QR
        Mat imageCamera = new Mat();

        zbarQR_obj = new zbarQR();
        float qr_offset = 0.1f;
        int qr_attemp = 0;

        while(zbarQR_obj.qrCodeString == null)
        {
            moveToWrapper( 11.11f + qr_offset , -9.8f, 4.79,0,0,-0.707,0.707);
            imageCamera = api.getMatNavCam();
            qr_attemp++;
            Log.d("QR",String.format("Start to read QR : %d",qr_attemp));
            zbarQR_obj.scanQRImage(imageCamera);

            qr_offset *= -1;
        }
        qr_offset *= -1;
        Log.d("QR","readed : " + zbarQR_obj.qrCodeString);
        api.sendDiscoveredQR(zbarQR_obj.qrCodeString);      //scan QR code

        StringDecode QRData = new StringDecode();
        QRData.setString(zbarQR_obj.qrCodeString);      //split decoded QR code
        Log.d("QR","End to read QR");
        Log.d("QR", String.format("QR A : %d %.2f %.2f %.2f",QRData.getPattern(),QRData.getPosX(),QRData.getPosY(),QRData.getPosZ()));

        //AR
//        relativeMoveToWrapper(0,0,10,0,0,-0.707,0.707);
        moveToWrapper( 11.01 , -9.8f, 4.79,0,0,-0.707,0.707);
        Mat ar_img = api.getMatNavCam();
        Kinematics kinec_takepic = api.getTrustedRobotKinematics();
        Point pos_takepic = kinec_takepic.getPosition();
        Log.d("AR", String.format("after move to AR x,y,z : %f %f %f", pos_takepic.getX(), pos_takepic.getY(), pos_takepic.getZ()));

        ARmodel ar_detector = new ARmodel();
        ar_detector.estimate(ar_img,camMatrix,dstMatrix);
//        if(ar_detector.getPosX() >= 0.08) {
//            Log.d("AR",String.format("real x : *f",ar_detector.getPosX()));
//            ar_detector.setPosX(0.08);
//        }
//        else if(ar_detector.getPosX() <= -0.08) {
//            Log.d("AR",String.format("real x : *f",ar_detector.getPosX()));
//            ar_detector.setPosX(-0.08);
//        }
        Point target_relative2 = new Point(ar_detector.getPosX(),ar_detector.getPosY(),ar_detector.getPosZ());
        Log.d("AR",String.format("real x : *f",ar_detector.getPosX()));
        Log.d("AR", String.format("target_relative2 x,y,z : %f %f %f", 0.08, target_relative2.getY(), target_relative2.getZ()));

        Quaternion rot_qua = alignX(target_relative2);

        kinec_takepic = api.getTrustedRobotKinematics();
        pos_takepic = kinec_takepic.getPosition();
        Log.d("AR", String.format("After alignX x,y,z : %f %f %f", pos_takepic.getX(), pos_takepic.getY(), pos_takepic.getZ()));
        Log.d("AR",String.format("rot qua : %f %f %f %f",rot_qua.getX(),rot_qua.getY(),rot_qua.getZ(),rot_qua.getW()));

        relativeMoveToWrapper(0,0,0,rot_qua.getX(),rot_qua.getY(),rot_qua.getZ(),rot_qua.getW());
        Log.d("AR","Aligned");

//        relativeMoveToWrapper(0,-0.0572,0.1111,rot_qua.getX(),rot_qua.getY(),rot_qua.getZ(),rot_qua.getW());

        kinec_takepic = api.getTrustedRobotKinematics();
        pos_takepic = kinec_takepic.getPosition();
        Log.d("AR", String.format("After adjust x,y,z : %f %f %f", pos_takepic.getX(), pos_takepic.getY(), pos_takepic.getZ()));

        //Laser Control
        api.laserControl(true);
        Log.d("QR", "laser");
        api.takeSnapshot();
        Log.d("QR", "take photo");

        moveToWrapper(10.505,-9,4.50, 0,0, -0.707, 0.707);
        moveToWrapper(10.505,-8.5,4.50, 0,0, -0.707, 0.707); // avoid KOZ2

        moveToWrapper(10.6, -8.0, 4.5,0, 0, -0.707, 0.707); //move to Point B
        Log.d("RE", "Reporting...");
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    public void moveToWrapper(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        final int LOOP_MAX = 10;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        Log.d("moveTo", String.format("Counter %d", loopCounter));
    }

    public void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                      double qua_x, double qua_y, double qua_z,
                                      double qua_w){
        final int LOOP_MAX = 2;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.relativeMoveTo(point, quaternion, false);
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            result = api.relativeMoveTo(point, quaternion, false);
            ++loopCounter;
        }
    }

    public Quaternion alignX(Point target)
    {
        double target_dist = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY() + target.getZ() * target.getZ());

        Point cosine_dir = new Point(target.getX()/target_dist,target.getY()/target_dist,target.getZ()/target_dist);
        double rot_angle = Math.acos(cosine_dir.getX());
        double u_y = cosine_dir.getZ() * - 1.0f / Math.sin(rot_angle); //i cross k
        double u_z = cosine_dir.getY() / Math.sin(rot_angle); //i cross j
        Point rot_axis = new Point(0,u_y,u_z);

        double qw = Math.cos(rot_angle/2);
        double qx = 0;
        double qy = Math.sin(rot_angle/2) * rot_axis.getY();
        double qz = Math.sin(rot_angle/2) * rot_axis.getZ();

        return new Quaternion((float)qx,(float)qy,(float)qz,(float)qw);
    }

    public Quaternion alignY(Point target)
    {
        double target_dist = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY() + target.getZ() * target.getZ());

        Point cosine_dir = new Point(target.getX()/target_dist,target.getY()/target_dist,target.getZ()/target_dist);
        double rot_angle = Math.acos(cosine_dir.getY());
        double u_x = cosine_dir.getZ() / Math.sin(rot_angle) * -1.0f; //-j cross k
        double u_y = 0; //j cross j
        double u_z = cosine_dir.getX() / Math.sin(rot_angle) ; //-j cross i
        Point rot_axis = new Point(u_x,u_y,u_z);

        double qw = Math.cos(rot_angle/2);
        double qx = Math.sin(rot_angle/2) * rot_axis.getX();
        double qy = 0;
        double qz = Math.sin(rot_angle/2) * rot_axis.getZ();

        return new Quaternion((float)qx,(float)qy,(float)qz,(float)qw);
    }

}

