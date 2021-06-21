package jp.jaxa.iss.kibo.rpc.defaultapk;



import android.util.Log;

import java.util.ArrayList;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;

public class ARmodel {

    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;


    private static float markerSize;

    private static Mat rvecs;
    private static Mat tvecs;

    private static Point3 target_pos=new Point3();

    private boolean isID(int index,int id) {
        return ids.get(index,0)[0] == id;
    }

    public float getPosX() {
        return (float)target_pos.x;
    }

    public float getPosY() {
        return (float)target_pos.z * (-1.0f);
    }

    public float getPosZ() {
        return (float)target_pos.y;
    }

    public void setPosZ(double value) {
        target_pos.y = value;
    }

    public void estimate(Mat img,Mat camMatrix,Mat dstMatrix) {


        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Aruco.detectMarkers(img ,dict ,corners,ids);

        if(ids.size().height>0) {

            markerSize=0.05f;
            rvecs = new Mat();
            tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerSize, camMatrix, dstMatrix, rvecs, tvecs);

            int id_cnt =0;

            for(int i=0;i<rvecs.size().height;i++) {

                id_cnt++;
                Mat rvec=new Mat(1,3,CvType.CV_64FC1);
                Mat tvec=new Mat(1,3,CvType.CV_64FC1);
                rvec.put(0, 0, rvecs.get(i,0));
                tvec.put(0, 0, tvecs.get(i,0));

                float pos_x=(float)tvec.get(0, 0)[0];
                float pos_y=(float)tvec.get(0, 1)[0];
                float pos_z=(float)tvec.get(0, 2)[0];


                float x_offset = 0.1125f;
                float y_offset = 0.0425f;

                if(isID(i,1)) {
                    x_offset *= -1;
                    y_offset *= -1;
                    Log.d("AR","found id 1");
                }
                else if(isID(i,2)) {
                    y_offset *= -1;
                    Log.d("AR","found id 2");
                }
                else if(isID(i,4)) {
                    x_offset *= -1;
                    Log.d("AR","found id 4");
                }
                else {
                    Log.d("AR","found id 3");
                }

                target_pos.x=target_pos.x+pos_x + x_offset;
                target_pos.y=target_pos.y+pos_y + y_offset;
                target_pos.z=target_pos.z+pos_z;
                Log.d("AR",String.format("pos_x : %f pos_y %f pos_z  %f",pos_x,pos_y,pos_z));
                for(int j=0;j<corners.size();j++) {
                    Log.d("AR",String.format("corner %d %d value : %f %f",i,j,corners.get(i).get(0, j)[0],corners.get(i).get(0, j)[1]));
                }

            }
            target_pos.x=target_pos.x/id_cnt;
            target_pos.y=target_pos.y/id_cnt;
            target_pos.z=target_pos.z/id_cnt;

        }
    }
}