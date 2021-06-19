package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.util.Log;

import org.apache.commons.lang.ObjectUtils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point3;

import java.util.ArrayList;

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
        Point3 QR_target = new Point3(11.21f, -9.8f, 4.79);
        moveToWrapper( QR_target.x,QR_target.y,QR_target.z,0,0,-0.707,0.707);
        Log.d("QR","Start to read QR");

        Mat imageCamera = api.getMatNavCam();
        String qr_str = decodeQR(imageCamera);
        StringDecode QRData = new StringDecode();
        QRData.setString(qr_str);
        Log.d("QR","End to read QR");
        Log.d("QR", String.format("QR A : %d %.2f %.2f %.2f",QRData.getPattern(),QRData.getPosX(),QRData.getPosY(),QRData.getPosZ()));


        moveToTarget(QRData.getPattern(),QRData.getPosX(),QRData.getPosY(),QRData.getPosZ());
        Log.d("move", "Move to A' task accomplished");

        Log.d("AR","Start AR function");

        Mat ar_pic = api.getMatNavCam();
        Kinematics kinec_takepic = api.getTrustedRobotKinematics();
        Point pos_takepic = kinec_takepic.getPosition();

        //ARUCO
        ARmodel ArucoModel = new ARmodel();
        ArucoModel.estimate(ar_pic,camMatrix,dstMatrix);
        Log.d("AR", String.format("AR relative : %.2f %.2f %.2f",ArucoModel.getPosX(),ArucoModel.getPosY(),ArucoModel.getPosZ()));

        Point target_point = new Point(pos_takepic.getX() + ArucoModel.getPosX(),pos_takepic.getY() + ArucoModel.getPosY(),pos_takepic.getZ() + ArucoModel.getPosZ());
        Log.d("AR",String.format("target point : %.3f %.3f %.3f",target_point.getX(),target_point.getY(),target_point.getZ()));

        Point target_relative = new Point(ArucoModel.getPosX() + 0.0994,ArucoModel.getPosY() - 0.0125,ArucoModel.getPosZ() - 0.0285);   //offset from NavCam to LaserPointer
        Log.d("AR",String.format("target relative : %f %f %f",target_relative.getX(),target_relative.getY(),target_relative.getZ()));
        Quaternion rot_qua = alignX(target_relative);
        Log.d("AR",String.format("rot qua : %f %f %f %f",rot_qua.getX(),rot_qua.getY(),rot_qua.getZ(),rot_qua.getW()));

        relativeMoveToWrapper(0,0,0,rot_qua.getX(),rot_qua.getY(),rot_qua.getZ(),rot_qua.getW());
        Log.d("AR","Aligned");


        Log.d("AR","Laser control activate");
        api.laserControl(true);
        Log.d("AR", "laser on");
        api.takeSnapshot();
        Log.d("AR", "take photo");
        api.laserControl(false);
        Log.d("AR", "laser off");



        Point AvoidKOZ2 = new Point(10.505,-9,4.50);
        moveOffTarget(QRData.getPattern(),AvoidKOZ2.getX(),AvoidKOZ2.getY(),AvoidKOZ2.getZ()); // avoid KOZ2
        moveToWrapper(10.505,-8.5,4.50, 0,0, -0.707, 0.707); // avoid KOZ2

        moveToWrapper(10.6, -8.0, 4.5,0, 0, -0.707, 0.707); //move to Point B



        api.reportMissionCompletion();
    }


    @Override
    protected void runPlan2(){

    }

    @Override
    protected void runPlan3(){

    }

    public String decodeQR(Mat qr_img)
    {
        String decoded = null;
        int loopCounter = 0;
        float x_offset = 0.1f;
        while (decoded == null){
            loopCounter++;
            zbarQR_obj = new zbarQR();
            zbarQR_obj.scanQRImage(qr_img);
            Log.d("QR",String.format("readed : %s loop_cnt : %d" ,zbarQR_obj.qrCodeString,loopCounter));
            api.sendDiscoveredQR(zbarQR_obj.qrCodeString);
            decoded = zbarQR_obj.qrCodeString;
            if(decoded == null){
                x_offset *= -1.0f;
                Kinematics KinecCurrent = api.getTrustedRobotKinematics();
                Point PosCurrent = KinecCurrent.getPosition();
                moveToWrapper(PosCurrent.getX() + x_offset,PosCurrent.getY(),PosCurrent.getZ(),0,0,-0.707,0.707);
            }
        }

        return decoded;
    }


    public void moveToWrapper(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        final int LOOP_MAX = 2;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);
        int loopCounter = 0;
        while(!result.hasSucceeded()|| loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
    }

    public void moveOffTarget(int koz_pattern,double px, double py, double pz){
        Log.d("move", String.format("Start move form A' to B"));

        Kinematics KinecTarget = api.getTrustedRobotKinematics();
        Point PosTarget = KinecTarget.getPosition();
        if((koz_pattern == 1) || (koz_pattern == 2) || (koz_pattern == 3) || (koz_pattern == 4) || (koz_pattern == 8) ){
            Log.d("move", String.format("Start move to avoid KOZ : (%.2f, %.2f, %.2f)", PosTarget.getX(), PosTarget.getY(), pz));
            moveToWrapper(PosTarget.getX(), PosTarget.getY(), pz, 0,0,-0.707,0.707);
        }
        else if((koz_pattern == 5) || (koz_pattern == 6)){
            Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", px, PosTarget.getY(), PosTarget.getZ()));
            moveToWrapper(px, PosTarget.getY(), PosTarget.getZ(), 0,0,-0.707,0.707);
        }
        else if(koz_pattern == 7){
            Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", PosTarget.getX() + 0.6f, PosTarget.getY(), PosTarget.getZ()));
            moveToWrapper(PosTarget.getX() +0.6, PosTarget.getY(), PosTarget.getZ(), 0,0,-0.707,0.707);
            Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", PosTarget.getX() +0.6f, PosTarget.getY(), pz));
            moveToWrapper(PosTarget.getX() +0.6, PosTarget.getY(), pz, 0,0,-0.707,0.707);

        }
        moveToWrapper(px, py, pz, 0,0,-0.707,0.707);
        Log.d("move", String.format("avoid KOZ complete", px, py, pz));

    }
    public void moveToTarget(int koz_pattern,double qr_x, double qr_y, double qr_z){

        Log.d("move", String.format("Start moveFromQR function pattern : %d", koz_pattern));

        boolean is_collusion =  moveToKOZ(qr_x, qr_y, qr_z, 0,0,-0.707,0.707);
        Kinematics KinecReadQR = api.getTrustedRobotKinematics();
        Point PosReadQR = KinecReadQR.getPosition();
        if(is_collusion){
            Log.d("move", "Collusion detected!");
            Log.d("move", String.format("Move by offset to A', KOZ Pattern : %d",koz_pattern));

//            String.format("Start move pattern : %d",koz_pattern)
            if((koz_pattern == 1) || (koz_pattern == 2) || (koz_pattern == 3) || (koz_pattern == 4) || (koz_pattern == 8) ){
                Log.d("move", String.format("Start move to avoid KOZ : (%.2f, %.2f, %.2f)", qr_x, qr_y, PosReadQR.getZ()));
                moveToWrapper(qr_x, qr_y, PosReadQR.getZ(), 0,0,-0.707,0.707);

                Log.d("move", String.format("Move to A' : (%.2f, %.2f, %.2f)", qr_x, qr_y, qr_z));
                moveToWrapper(qr_x, qr_y, qr_z, 0,0,-0.707,0.707);
            }
            else if((koz_pattern == 5) || (koz_pattern == 6)){
                Log.d("move", String.format("Start move to avoid KOZ : (%.2f - 0.6, %.2f, %.2f)", qr_x, qr_y, PosReadQR.getZ()));
                moveToWrapper(qr_x - 0.6, qr_y, PosReadQR.getZ(), 0,0,-0.707,0.707);

                Log.d("move", String.format("Start move to avoid KOZ : (%.2f - 0.6, %.2f, %.2f)", qr_x, qr_y, qr_z));
                moveToWrapper(qr_x - 0.6, qr_y, qr_z, 0,0,-0.707,0.707);

                Log.d("move", String.format("Move to A' : (%.2f, %.2f, %.2f)", qr_x, qr_y, qr_z));
                moveToWrapper(qr_x, qr_y, qr_z, 0,0,-0.707,0.707);
            }
            else if(koz_pattern == 7){
                Log.d("move", String.format("Check move to avoid KOZ : (%.2f , %.2f, %.2f)", qr_x + 0.6, qr_y, PosReadQR.getZ()));
                boolean koz7_r_collusion =  moveToKOZ(qr_x + 0.6, qr_y, PosReadQR.getZ(), 0,0,-0.707,0.707);
                if (koz7_r_collusion){
                    Log.d("move", "Collusion detect! in pattern 7 going a roundabout way");

                    Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", qr_x - 0.75, qr_y, PosReadQR.getZ()));
                    moveToWrapper(qr_x - 0.75, qr_y, PosReadQR.getZ(), 0,0,-0.707,0.707);

                    Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", qr_x - 0.75, qr_y, qr_z + 0.6));
                    moveToWrapper(qr_x - 0.75, qr_y, qr_z + 0.6, 0,0,-0.707,0.707);

                    Log.d("move", String.format("Start move to avoid KOZ : (%.2f , %.2f, %.2f)", qr_x, qr_y, qr_z + 0.6));
                    moveToWrapper(qr_x, qr_y , qr_z + 0.6, 0,0,-0.707,0.707);

                    Log.d("move", String.format("Move to A' : (%.2f, %.2f, %.2f)", qr_x, qr_y, qr_z));
                    moveToWrapper(qr_x, qr_y, qr_z, 0,0,-0.707,0.707);
                }else{
                    Log.d("move", "No collusion detect! in pattern 7 try to get through");

                    Log.d("move", String.format("Start move to avoid KOZ : (%.2f + 0.6, %.2f, %.2f)", qr_x, qr_y, qr_z));
                    moveToWrapper(qr_x + 0.6, qr_y, qr_z, 0,0,-0.707,0.707);

                    Log.d("move", String.format("Move to A' : (%.2f, %.2f, %.2f)", qr_x, qr_y, qr_z));
                    moveToWrapper(qr_x , qr_y, qr_z, 0,0,-0.707,0.707);
                }

            }



        }
        else{
            Log.d("move", "No collusion detected");
            Log.d("move", String.format("Start to move directly to target, KOZ Pattern : %d",koz_pattern));

        }

    }

    public boolean moveToKOZ(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);


        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX ){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
            if (result.getStatus() ==  Result.Status.EXEC_FAILED ) {
                Log.d("move", String.format("collusion = true (NULL) loop_cnt: %d", loopCounter));
                return true;
            }
        }

        return false;



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
        double u_y = cosine_dir.getZ() * - 1.0f; //i cross k
        double u_z = cosine_dir.getY(); //i cross j
        Point rot_axis = new Point(0,u_y,u_z);

        double qw = Math.cos(rot_angle/2);
        double qx = 0;
        double qy = Math.sin(rot_angle/2) * rot_axis.getY();
        double qz = Math.sin(rot_angle/2) * rot_axis.getZ();

        return new Quaternion((float)qx,(float)qy,(float)qz,(float)qw);
    }

}

