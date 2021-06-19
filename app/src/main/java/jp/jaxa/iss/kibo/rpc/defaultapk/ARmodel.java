package jp.jaxa.iss.kibo.rpc.defaultapk;



import java.util.ArrayList;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;


public class ARmodel {

    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;
    private static Scalar borderColor;


    private static float markerSize;

    private static Mat rvecs;
    private static Mat tvecs;

    private static Mat rotationMatrix;

    private static Point3 target_pos=new Point3();

    private boolean isID(int index,int id) {
        return ids.get(index,0)[0] == id;
    }

    public float getPosX() { return (float)target_pos.x;}

    public float getPosY() {
        return (float)target_pos.z * (-1.0f);
    }

    public float getPosZ() {
        return (float)target_pos.y;
    }


    public void estimate(Mat img,Mat camMatrix,Mat dstMatrix) {


        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Aruco.detectMarkers(img ,dict ,corners,ids);

//        borderColor = new Scalar (255.0, 0.0, 0.0);

        if(ids.size().height>0) {

//			Aruco.drawDetectedMarkers(img , corners, ids, borderColor);		// draw square @ Markers

            markerSize=0.05f;
            rotationMatrix = new Mat ();
            rvecs = new Mat();
            tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerSize, camMatrix, dstMatrix, rvecs, tvecs);


            int id_cnt =0;


            for(int i=0;i<rvecs.size().height;i++) {

                id_cnt++;
                Mat rvec=new Mat(1,3,CvType.CV_64FC1);
                Mat tvec=new Mat(1,3,CvType.CV_64FC1);
                Mat rot=new Mat();
                rvec.put(0, 0, rvecs.get(i,0));
                tvec.put(0, 0, tvecs.get(i,0));

//				Calib3d.drawFrameAxes(img, camMatrix, dstMatrix, rvec, tvec, (float)0.15); 	// draw 3axis @ Markers


                float pos_x=(float)tvec.get(0, 0)[0];
                float pos_y=(float)tvec.get(0, 1)[0];
                float pos_z=(float)tvec.get(0, 2)[0];


                float x_offset = 0.1125f;
                float y_offset = 0.0415f;

                if(isID(i,1)) {
                    x_offset *= -1;
                    y_offset *= -1;
                }
                else if(isID(i,2)) {
                    y_offset *= -1;
                }
                else if(isID(i,4)) {
                    x_offset *= -1;
                }

                target_pos.x=target_pos.x+pos_x + x_offset;
                target_pos.y=target_pos.y+pos_y + y_offset;
                target_pos.z=target_pos.z+pos_z;

            }

            target_pos.x=target_pos.x/id_cnt;
            target_pos.y=target_pos.y/id_cnt;
            target_pos.z=target_pos.z/id_cnt;

        }
    }
}