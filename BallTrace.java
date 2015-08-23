package com.example.balltrace;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.MatOfRect;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;

//import com.learn2crack.speech.MainActivity;
//import com.learn2crack.speech.R;


//import com.learn2crack.speech.R;


import android.app.Activity;
import android.app.ActionBar;
import android.app.Dialog;
import android.app.Fragment;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.Toast;
import android.widget.TextView;
import android.widget.Button;
import android.view.Window;

import java.net.ServerSocket;
import java.net.Socket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.BufferedWriter;

import android.speech.RecognizerIntent;

import java.util.ArrayList;

import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
/*
public class BallTrace extends Activity {
    private static final String TAG = "ObjTrackActivity";
    
    private MenuItem mItemPreviewRGBA;
    private MenuItem mItemPreviewTresholded;
    
    public static boolean bShowTresholded = false;

    public BallTrace() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    // Called when the activity is first created. 
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(new ObjTrackView(this));
    }
    
    public boolean onCreateOptionsMenu(Menu menu){
    	Log.i(TAG, "onCreateOptionsMenu");
    	mItemPreviewRGBA = menu.add("Preview RGBA");
    	mItemPreviewTresholded = menu.add("Preview Thresholded");
    	return true;
    }
    
    public boolean onOptionsItemSelected(MenuItem item){
    	Log.i(TAG, "Menu Item selected " + item);
    	if (item == mItemPreviewRGBA)
    		bShowTresholded = false;
        else if (item == mItemPreviewTresholded)
        	bShowTresholded = true;
    	return true;
    }
}
*/


public class BallTrace extends Activity implements CvCameraViewListener2 {
    private static final String TAG = "OCVSample::Activity";

    private Mat             mRgba;
    private Mat             mGray;
    private Mat				mOut;
    private int				m_width;
    private int 			m_height;
    private CameraBridgeViewBase mOpenCvCameraView;
    private Button button;
    private Button button_standby;
    private Button button_poweroff;
    private Button button_talk;
    private Button button_forward;
    private Button button_backward;
    private Button button_stop;
    
    private SensorManager mSensorManager;
    private Sensor mPressure;
    private Sensor mAccelerometer;
    private Sensor mProximity;
    
    int [] BallCenter;
    int [] DoorLocation;
    
    private static final int REQUEST_CODE = 1234;
    TextView Speech;
    Dialog match_text_dialog;
    ListView textlist;
    ArrayList<String> matches_text;
    
    Thread detectThread = null;
    
    private Socket socket;
    Thread serverThread = null;
    public static final int SERVERPORT = 6000;
    private static final String SERVER_IP = "192.168.2.77";

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    // Load native library after(!) OpenCV initialization
                    System.loadLibrary("objtrack_opencv_jni");
                    
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_ball_trace);
		
	    BallCenter = new int[3];
	    DoorLocation = new int[2];
	    
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.balltrace_activity_java_surface_view);

        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);

        mOpenCvCameraView.setCvCameraViewListener(this);
/*		
        button = (Button) findViewById(R.id.button1);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                send_message("<cmd>kick</cmd>");
            }
        });

        button_standby = (Button) findViewById(R.id.button2);
        button_standby.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
                send_message("<cmd>standby</cmd>");
             }
        });
        
        button_poweroff = (Button) findViewById(R.id.button3);
        button_poweroff.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
             	send_message("<cmd>poweroff</cmd>");
            }
        });
        
        button_forward = (Button) findViewById(R.id.button_forward);
        button_forward.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
             	send_message("<cmd>forward</cmd>");
            }
        });        

        button_backward = (Button) findViewById(R.id.button_backward);
        button_backward.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
             	send_message("<cmd>backward</cmd>");
            }
        });
        
        button_stop = (Button) findViewById(R.id.button_stop);
        button_stop.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
             	send_message("<cmd>stop</cmd>");
            }
        });        
        
        
        button_talk = (Button) findViewById(R.id.button_talk);
        button_talk.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Perform action on click
            	if(isConnected()){
               	 	Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
               	 		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
               	 		RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
               	 		startActivityForResult(intent, REQUEST_CODE);
               	}
               	else{
               		Toast.makeText(getApplicationContext(), "Plese Connect to Internet", Toast.LENGTH_LONG).show();
               	}
            }
        });        
  */      
        new Thread(new ClientThread()).start();
        
        new Thread(new DetectThread()).start();
        
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        //List<Sensor> deviceSensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);        
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (mAccelerometer != null) 
        {
            mSensorManager.registerListener(mySensorEventListener, mAccelerometer,
                SensorManager.SENSOR_DELAY_NORMAL);
            Log.i("Compass MainActivity", "Registerered for ORIENTATION Sensor");
        } 
        else 
        {
            Log.e("Compass MainActivity", "Registerered for ORIENTATION Sensor");
            Toast.makeText(this, "ORIENTATION Sensor not found",
                Toast.LENGTH_LONG).show();
            finish();
          }
        
        if( mAccelerometer == null )
        {
   
        }
        //
        mProximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
        if (mProximity != null) 
        {
            mSensorManager.registerListener(mySensorProximityEventListener, mProximity,
                SensorManager.SENSOR_DELAY_NORMAL);
            Log.i("Compass MainActivity", "Registerered for ORIENTATION Sensor");
        } 
        else 
        {
            Log.e("Compass MainActivity", "Registerered for ORIENTATION Sensor");
            Toast.makeText(this, "ORIENTATION Sensor not found",
                Toast.LENGTH_LONG).show();
            finish();
          }
        
        if( mProximity == null )
        {
   
        }

	}
	 private SensorEventListener mySensorProximityEventListener = new SensorEventListener() {

		    @Override
		    public void onAccuracyChanged(Sensor sensor, int accuracy) {
		    }

		    @Override
		    public void onSensorChanged(SensorEvent event) {
		      // angle between the magnetic north direction
		      // 0=North, 90=East, 180=South, 270=West
		      float azimuth = event.values[0];
		      azimuth += 1;
//		      compassView.updateData(azimuth);
		    }
		  };
	
	 private SensorEventListener mySensorEventListener = new SensorEventListener() {

		    @Override
		    public void onAccuracyChanged(Sensor sensor, int accuracy) {
		    }

		    @Override
		    public void onSensorChanged(SensorEvent event) {
		      // angle between the magnetic north direction
		      // 0=North, 90=East, 180=South, 270=West
		      float azimuth = event.values[0];
		      azimuth += 1;
//		      compassView.updateData(azimuth);
		    }
		  };
	public int send_message(String str)
	{     
        try {
          	PrintWriter out = new PrintWriter(new BufferedWriter(
        	                 new OutputStreamWriter(socket.getOutputStream())),
        	                  true);
        	out.println(str);
         } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }            	
		return 0;
	}
	
	public boolean isConnected()
	{ 
		ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
		NetworkInfo net = cm.getActiveNetworkInfo();
		if (net!=null && net.isAvailable() && net.isConnected()) {
			return true;
		} else {
			return false; 
		}
	}
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    	if (requestCode == REQUEST_CODE && resultCode == RESULT_OK) {
       		matches_text = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
	   		String str;
// 			Speech.setText("You have said " +matches_text.get(position));
    		str = "<cmd>speech:"+ matches_text.get(0)+"</cmd>";
			send_message(str);
			
    		/*
     		match_text_dialog = new Dialog(BallTrace.this);
    		match_text_dialog.setContentView(R.layout.dialog_matches_frag);
    		match_text_dialog.setTitle("Select Matching Text");
    		textlist = (ListView)match_text_dialog.findViewById(R.id.list);
    		matches_text = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
    		ArrayAdapter<String> adapter = new ArrayAdapter<String>(this,android.R.layout.simple_list_item_1, matches_text);
    		textlist.setAdapter(adapter);
    		textlist.setOnItemClickListener(new AdapterView.OnItemClickListener() {
     //@Override
     			public void onItemClick(AdapterView<?> parent, View view,int position, long id) {
     		   		String str;
//    	 			Speech.setText("You have said " +matches_text.get(position));
     	    		str = "<cmd>speech:"+ matches_text.get(0)+"</cmd>";
     				send_message(str);		

     				match_text_dialog.hide();
     			}
    		});
    		match_text_dialog.show();
    		*/
    	}
    	
    	super.onActivityResult(requestCode, resultCode, data);
    }
	
    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        
        if (mAccelerometer != null) {
            mSensorManager.unregisterListener(mySensorEventListener);
          }
        
    }

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {

		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.ball_trace, menu);
		return true;
	}

    public void onCameraViewStarted(int width, int height) {    	
        mGray = new Mat();
        mRgba = new Mat();
        mOut = new Mat();
    
        m_width = width;
        m_height = height;
        Log.i(TAG, "Width="+width+"height"+height);
    }

    
    public void onCameraViewStopped() {
    	mGray.release();
    	mRgba.release();
    	mOut.release();
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
    	Point pt1 = new Point(0,0);
    	Point pt2 = new Point(200,200);
    	Scalar color = new Scalar(0,255,255);
    	int [] retArray;
    	retArray = new int[5];
    	retArray[0] = 0;
    	retArray[1] = 0;
     	
        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();
        int depth = mRgba.depth();
        int channels = mRgba.channels();
        
        int depth1 = mGray.depth();
        int channels1 = mGray.channels();
        
        //mOut = inputFrame.rgba();
      retArray = BallDetect(m_width, m_height, mGray.getNativeObjAddr(), mRgba.getNativeObjAddr(), mOut.getNativeObjAddr());
//        retArray = MotionDetect(m_width, m_height, mGray.getNativeObjAddr(), mRgba.getNativeObjAddr(), mOut.getNativeObjAddr());
        //retArray = DoorDetect(m_width, m_height, mGray.getNativeObjAddr(), mRgba.getNativeObjAddr(), mOut.getNativeObjAddr());
        
        Log.i(TAG, "center x="+retArray[0]+" y="+retArray[1]+" r="+retArray[2]);

        BallCenter[0] = retArray[0];	// X
        BallCenter[1] = retArray[1];	// Y
        BallCenter[2] = retArray[2];	// r
        
        DoorLocation[0] = retArray[3];
        DoorLocation[1] = retArray[4];
                
        pt1.x = 1100;
        pt1.y = 0;
        pt2.x = 1100;
        pt2.y = 1080;
        Core.line(mRgba, pt1, pt2, color,2);
        pt1.x = 0;
        pt1.y = 420;
        pt2.x = 1920;
        pt2.y = 420;
        Core.line(mRgba, pt1, pt2, color,2);
        
        return mRgba;
    }
    
    class DetectThread implements Runnable {
    	public void run() {
    		
    		int ball_x;
    		int ball_y;
    		int ball_r;
    		int need_send_ball_pos;
    		ball_x = 0;
    		ball_y = 0;
    		ball_r = 0;
    		need_send_ball_pos = 0;
    		
    		while( true) {
    			try{
    				//if(BallCenter[0] > 200)
    		        //Speech = (TextView)findViewById(R.id.speech);
    		       //Speech.setText("ccc="+BallCenter[0]);
    				String str;
    				if( (ball_x != BallCenter[0]) || (ball_y != BallCenter[1]) || (ball_r != BallCenter[2]) )
    				{
    					need_send_ball_pos = 1;
    					ball_x = BallCenter[0];
    					ball_y = BallCenter[1];
    					ball_r = BallCenter[2];
    				}
    				if( need_send_ball_pos == 1 )
    				{
    					str = "<cmd>position="+BallCenter[0]+";"+BallCenter[1]+";"+BallCenter[2]+"</cmd>";    						
    					send_message(str);
    					need_send_ball_pos = 0;
   				        //Log.i(TAG, "send ="+str);
    				}
      				
    				str = "<cmd>doorloc="+DoorLocation[0]+";"+ DoorLocation[1]+"</cmd>";
    				send_message(str);
    				
    				Thread.sleep(200,0);
    			} catch (InterruptedException e) {
    				System.out.println("Thread " + " interrupted.");
    			}
     		}
    	}
    }
    
   
    class ClientThread implements Runnable {
    	@Override
    	public void run() {
    		try {
    			InetAddress serverAddr = InetAddress.getByName(SERVER_IP);
    	        socket = new Socket(serverAddr, SERVERPORT);
    	        Log.i(TAG, "open socket");
     	    } catch (UnknownHostException e1) {
     	        e1.printStackTrace();
    	    } catch (IOException e1) {
    	        e1.printStackTrace();
     	    }
    	}
    }

    public native int[] BallDetect(int width, int height, long yuv, long rgba, long out);
//    public native int[] MotionDetect(int width, int height, long yuv, long rgba, long out);
//    public native int[] DoorDetect(int width, int height, long yuv, long rgba, long out);
}
