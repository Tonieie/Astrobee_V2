package jp.jaxa.iss.kibo.rpc.defaultapk;


import org.opencv.core.Mat;

public class quaternion {
	public static float x=0;
	public static float y=0;
	public static float z=0;
	public static float w=1;
	
	public static float roll=0;
	public static float pitch=0;
	public static float yaw=0;
	private static float M_PI=3.1416f;
	
	public static void toEuler() {
		// roll (x-axis rotation)
	    double sinr_cosp = 2 * (w * x + y * z);
	    double cosr_cosp = 1 - 2 * (x * x + y * y);
	    roll = (float) Math.atan2(sinr_cosp, cosr_cosp);

	    // pitch (y-axis rotation)
	    float sinp = 2 * (w * y - z * x);
	    if (Math.abs(sinp) >= 1)
	       pitch = Math.copySign(M_PI / 2, sinp); // use 90 degrees if out of range
	    else
	       pitch = (float) Math.asin(sinp);

	    // yaw (z-axis rotation)
	    double siny_cosp = 2 * (w * z + x * y);
	    double cosy_cosp = 1 - 2 * (y * y + z * z);
	    yaw = (float) Math.atan2(siny_cosp, cosy_cosp);
	}
	
 	public static void fromRotationMatrix(Mat rot) {
    	int row=3;
    	int col=3;
    	float[][] m=new float[3][3];

    	for(int i=0;i<row;i++) {
    		for(int j=0;j<col;j++) {
    			m[i][j]=(float) rot.get(i,j)[0];
    		}		
    	}
    	float tr=m[0][0]+m[1][1]+m[2][2];
    			
    	if(tr>0.0) {
    		float S = (float)Math.sqrt((float)tr+1.0) * 2;
    		w=(float)0.25*S;
    		x = (m[2][1] - m[1][2]) / S;
    		y = (m[0][2] - m[2][0]) / S; 
    	    z = (m[1][0] - m[0][1]) / S; 
    	}    	
    	else if((m[0][0]>m[1][1])&(m[0][0]>m[2][2])) {
    		float S = (float)Math.sqrt(1.0+m[0][0]-m[1][1]-m[2][2]) * 2;
    		w=(m[2][1] - m[1][2]) / S;
    		x=(float)0.25*S;
    		y = (m[0][1] + m[1][0]) / S; 
    		z = (m[0][2] + m[2][0]) / S;
    	}
    	else if (m[1][1] > m[2][2]) { 
    		float S = (float)Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
    		w = (m[0][2] - m[2][0]) / S;
    		x = (m[0][1] + m[1][0]) / S; 
    		y = (float)0.25 * S;
    		z = (m[1][2] + m[2][1]) / S; 
    	} 
    	else { 
    		float S = (float)Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
    		w = (m[1][0] - m[0][1]) / S;
    		x = (m[0][2] + m[2][0]) / S;
    		y = (m[1][2] + m[2][1]) / S;
    		z = (float)0.25 * S;
    	}
    }
}
