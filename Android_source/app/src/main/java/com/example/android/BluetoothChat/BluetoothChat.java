package com.example.android.BluetoothChat;


import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.location.Address;
import android.location.Criteria;
import android.location.Geocoder;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Vibrator;
import android.util.Log;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.webkit.WebView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/**
 * This is the main Activity that displays the current chat session.
 */
public class BluetoothChat extends Activity {
    // Debugging
    private static final String TAG = "BluetoothChat";
    private static final boolean D = true;

    // Message types sent from the BluetoothChatService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;

    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";

    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;

    // Layout Views
    private TextView mTitle;
    private ListView mConversationView;
    private EditText mOutEditText;
    private Button mSendButton;
    private  TextView dustText;
    private WebView weather;
    private TextView adressText;
    TextView avgText;
    LocationClass locationClass;


    int nowDust=0;

    // Name of the connected device
    private String mConnectedDeviceName = null;
    // Array adapter for the conversation thread
    private ArrayAdapter<String> mConversationArrayAdapter;
    // String buffer for outgoing messages
    private StringBuffer mOutStringBuffer;
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    // Member object for the chat services
    private BluetoothChatService mChatService = null;


    ////////////////////////////GPS를 다루기 위한 부분////////////////////////////////////////////


    double lat;
    double lot;
    LocationManager mLocMgr;

    /////////////////////////////////////////////////////////////////////////////////

String magimak = "";
    ///////////////////////////////////////////////

    long  avgDust = 0;
    long  avgDustCnt = 0;
    int lastDust = 0;
    //////////////////////////////

    public static Criteria getCriteria() {
        Criteria criteria = new Criteria();
        criteria.setAccuracy(Criteria.ACCURACY_FINE);
        criteria.setAltitudeRequired(true);
        criteria.setBearingRequired(true);
        criteria.setSpeedRequired(true);
        criteria.setCostAllowed(true);
        criteria.setPowerRequirement(Criteria.POWER_HIGH);
        return criteria;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if(D) Log.e(TAG, "+++ ON CREATE +++");

        // Set up the window layout
        requestWindowFeature(Window.FEATURE_CUSTOM_TITLE);
        setContentView(R.layout.main);
        getWindow().setFeatureInt(Window.FEATURE_CUSTOM_TITLE, R.layout.custom_title);


        avgText = (TextView)findViewById(R.id.avg);


        // Set up the custom title
        mTitle = (TextView) findViewById(R.id.title_left_text);
        mTitle.setText(R.string.app_name);
        mTitle = (TextView) findViewById(R.id.title_right_text);
        dustText = (TextView)findViewById(R.id.dust);
        weather = (WebView)findViewById(R.id.weather);
        adressText = (TextView)findViewById(R.id.adress);

        weather.getSettings().setJavaScriptEnabled(true);
        weather.setScrollY(235);
        weather.setHorizontalScrollBarEnabled(false);
        weather.setVerticalScrollBarEnabled(false);

        weather.loadUrl("https://m.search.naver.com/search.naver?where=m&sm=mtb_she&qdt=0&query=%EC%A1%B0%EC%B9%98%EC%9B%90%EC%9D%8D%20%EB%82%A0%EC%94%A8");            // 웹뷰에서 불러올 URL 입력

        mLocMgr = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
        String locProv = mLocMgr.getBestProvider(getCriteria(), true);

        mLocMgr.requestLocationUpdates( LocationManager.NETWORK_PROVIDER, 3000, 3, mLocListener );

        //dustText.setText(nowDust);


        // Get local Bluetooth adapter
        //블루투스 어뎁터 생성.
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
    }

    void changeWICHI(double lat, double lot){

        String wichi = getLocation(lat, lot);


        String[] dasida = wichi.split(" ");
        String realwichi = dasida[3] + " "+ dasida[4] +" " + dasida[5];
        String realwichi2 = dasida[4] +" " + dasida[5];
        adressText.setText(realwichi);

        if(magimak != realwichi2){
            weather.loadUrl("https://m.search.naver.com/search.naver?where=m&sm=mtb_she&qdt=0&query=%EC%A1%B0%EC%B9%98%EC%9B%90%EC%9D%8D%20%EB%82%A0%EC%94%A8");            // 웹뷰에서 불러올 URL 입력

            mConversationArrayAdapter.add(realwichi2+"의 미세먼지 평균수치:  " + avgDust/avgDustCnt+"㎍/㎥");
            magimak = realwichi2;

        }




    }
    LocationListener mLocListener= new LocationListener() {
        @Override
        public void onLocationChanged(Location location) {
          //  mConversationArrayAdapter.add("GPS값:  " + nowDust+","+location.getLatitude());
            //mConversationArrayAdapter.add("GPS값:  " + nowDust+","+location.getLongitude());

            changeWICHI(location.getLatitude(),location.getLongitude());




        }

        @Override
        public void onStatusChanged(String provider, int status, Bundle extras) {

        }

        @Override
        public void onProviderEnabled(String provider) {

        }

        @Override
        public void onProviderDisabled(String provider) {

        }
    };

    @Override
    public void onStart() {
        super.onStart();
        if(D) Log.e(TAG, "++ ON START ++");

        // If BT is not on, request that it be enabled.
        // setupChat() will then be called during onActivityResult
        //블루투스 활성화 확인.
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
        // Otherwise, setup the chat session
        } else {
            if (mChatService == null) setupChat();
        }
    }

    @Override
    public synchronized void onResume() {
        super.onResume();
        if(D) Log.e(TAG, "+ ON RESUME +");

        // Performing this check in onResume() covers the case in which BT was
        // not enabled during onStart(), so we were paused to enable it...
        // onResume() will be called when ACTION_REQUEST_ENABLE activity returns.
        if (mChatService != null) {
            // Only if the state is STATE_NONE, do we know that we haven't started already
            if (mChatService.getState() == BluetoothChatService.STATE_NONE) {
              // Start the Bluetooth chat services
              mChatService.start();
            }
        }
    }

    private void setupChat() {
        Log.d(TAG, "setupChat()");
        weather.getSettings().setJavaScriptEnabled(true);
        weather.setScrollY(235);
        weather.setHorizontalScrollBarEnabled(false);
        weather.setVerticalScrollBarEnabled(false);

        weather.loadUrl("https://m.search.naver.com/search.naver?where=m&sm=mtb_she&qdt=0&query=%EC%A1%B0%EC%B9%98%EC%9B%90%EC%9D%8D%20%EB%82%A0%EC%94%A8");
        // Initialize the array adapter for the conversation thread
        mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message);
        mConversationView = (ListView) findViewById(R.id.in);
        mConversationView.setAdapter(mConversationArrayAdapter);

        // Initialize the compose field with a listener for the return key
        mOutEditText = (EditText) findViewById(R.id.edit_text_out);
        mOutEditText.setOnEditorActionListener(mWriteListener);

        // Initialize the send button with a listener that for click events
        mSendButton = (Button) findViewById(R.id.button_send);
        mSendButton.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                // Send a message using content of the edit text widget
                TextView view = (TextView) findViewById(R.id.edit_text_out);
                String message = view.getText().toString();
                sendMessage(message);
            }
        });

        // Initialize the BluetoothChatService to perform bluetooth connections
        mChatService = new BluetoothChatService(this, mHandler);

        // Initialize the buffer for outgoing messages
        mOutStringBuffer = new StringBuffer("");
    }

    @Override
    public synchronized void onPause() {
        super.onPause();
        if(D) Log.e(TAG, "- ON PAUSE -");
    }

    @Override
    public void onStop() {
        super.onStop();
        if(D) Log.e(TAG, "-- ON STOP --");
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // Stop the Bluetooth chat services
        if (mChatService != null) mChatService.stop();
        if(D) Log.e(TAG, "--- ON DESTROY ---");
    }

    private void ensureDiscoverable() {    	
        if(D) Log.d(TAG, "ensure discoverable");
        //스켄 모드 설정. 
        
        if (mBluetoothAdapter.getScanMode() != BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE) {
            Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
            discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
            startActivity(discoverableIntent);
        }
    }

    /**
     * Sends a message.
     * @param message  A string of text to send.
     */
    private void sendMessage(String message) {
        // Check that we're actually connected before trying anything
    	//연결상태를 확인하고 연결 되어있다면 메시지를 전송.
    	
        if (mChatService.getState() != BluetoothChatService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

        // Check that there's actually something to send
        if (message.length() > 0) {
            // Get the message bytes and tell the BluetoothChatService to write
            byte[] send = message.getBytes();
            mChatService.write(send);

            // Reset out string buffer to zero and clear the edit text field
            mOutStringBuffer.setLength(0);
            mOutEditText.setText(mOutStringBuffer);
        }
    }

    // The action listener for the EditText widget, to listen for the return key
    private TextView.OnEditorActionListener mWriteListener = new TextView.OnEditorActionListener() {
    	
        public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
            // If the action is a key-up event on the return key, send the message
        	
            if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
                String message = view.getText().toString();
                sendMessage(message);
            }
            if(D) Log.i(TAG, "END onEditorAction");
            return true;
        }
    };


    //////////////////////////GPS/////////////////////

    public void checkMyLocation() {


            locationClass.onResumeLocation();


    }

    public String getLocation(double lat, double lng){
        String str = null;
        Geocoder geocoder = new Geocoder(this, Locale.KOREA);

        List<Address> address;
        try {
            if (geocoder != null) {
                address = geocoder.getFromLocation(lat, lng, 1);
                if (address != null && address.size() > 0) {
                    str = address.get(0).getAddressLine(0).toString();
                }
            }
        } catch (IOException e) {
            Log.e("MainActivity", "주소를 찾지 못하였습니다.");
            e.printStackTrace();
        }

       return str;

    }


    /////////////////////////////////////////////////

    // The Handler that gets information back from the BluetoothChatService
    private final Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
            
            case MESSAGE_STATE_CHANGE:
            	
                if(D) Log.i(TAG, "MESSAGE_STATE_CHANGE: " + msg.arg1);
                switch (msg.arg1) {
                case BluetoothChatService.STATE_CONNECTED:
                    mTitle.setText(R.string.title_connected_to);
                    mTitle.append(mConnectedDeviceName);
                    mConversationArrayAdapter.clear();
                    break;
                    
                case BluetoothChatService.STATE_CONNECTING:
                    mTitle.setText(R.string.title_connecting);
                    break;
                    
                case BluetoothChatService.STATE_LISTEN:
                case BluetoothChatService.STATE_NONE:
                    mTitle.setText(R.string.title_not_connected);
                    break;
                    
                }
                break;
                
            case MESSAGE_WRITE:
                byte[] writeBuf = (byte[]) msg.obj;
                // construct a string from the buffer
                String writeMessage = new String(writeBuf);
                mConversationArrayAdapter.add("Me:  " + writeMessage);
                break;
                
            case MESSAGE_READ: /// 겟메시지
                byte[] readBuf = (byte[]) msg.obj;
                // construct a string from the valid bytes in the buffer
                String readMessage = new String(readBuf, 0, msg.arg1);
              //  mConversationArrayAdapter.add(mConnectedDeviceName+":  " + readMessage);
                ////////////////////////////////////////////////추가한 부분



                int count = 0;
                String regex = "^\\{\\d{1,3}\\}"; // {} 안에 숫자가 있는지 검사하는 정규식
                if(Pattern.matches(regex,readMessage)){
                  // checkMyLocation();
                    //mConversationArrayAdapter.add("lat:"+lat+"lot"+lot);
                   // mConversationArrayAdapter.add(getLocation(lat,lot));
                    nowDust = Integer.parseInt(readMessage.replaceAll("[^0-999]", "")); // {} 로 감싸져 있는 숫자를 가져옴
                   // mConversationArrayAdapter.add("나우더스트값:  " + nowDust+","+readMessage);
                    //checkMyLocation();
                    if(lastDust != nowDust){
                        avgDust += nowDust;
                        avgDustCnt++;
                        lastDust = nowDust;
                        avgText.setText(avgDust/avgDustCnt+"㎍/㎥");


                    }
                    if(nowDust<=30){
                        dustText.setTextColor(Color.rgb(34,177,76));
                    }else if(nowDust<=80){
                        dustText.setTextColor(Color.rgb(255,255,0));
                    }else if(nowDust<=150){
                        dustText.setTextColor(Color.rgb(255,127,39));
                    }else{
                        dustText.setTextColor(Color.rgb(255,0,0));
                    }

                    dustText.setText(Integer.toString(nowDust));
                }


                //Vibrator vibe = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);
               // vibe.vibrate(1500);



                /*

                    ~30 좋음
                    ~80 보통
                    ~150 나쁨
                    존나 나쁨


                 */









                //////////////////////////////////////////////////////
                break;
                
            case MESSAGE_DEVICE_NAME:
                // save the connected device's name
                mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
                Toast.makeText(getApplicationContext(), "Connected to "
                               + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
                break;
                
            case MESSAGE_TOAST:
                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
                               Toast.LENGTH_SHORT).show();
                break;
            }
        }
    };

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(D) Log.d(TAG, "onActivityResult " + resultCode);
        switch (requestCode) {
        
        case REQUEST_CONNECT_DEVICE:
            // When DeviceListActivity returns with a device to connect
            if (resultCode == Activity.RESULT_OK) {
                // Get the device MAC address
                String address = data.getExtras().getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
                // Get the BLuetoothDevice object
                BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                // Attempt to connect to the device
                mChatService.connect(device);
            }
            break;
            
        case REQUEST_ENABLE_BT:
            // When the request to enable Bluetooth returns
            if (resultCode == Activity.RESULT_OK) {
                // Bluetooth is now enabled, so set up a chat session
                setupChat();
            } else {
                // User did not enable Bluetooth or an error occured
                Log.d(TAG, "BT not enabled");
                Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.option_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.scan:
            // Launch the DeviceListActivity to see devices and do scan
            Intent serverIntent = new Intent(this, DeviceListActivity.class);
            startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
            return true;
        case R.id.discoverable:
            // Ensure this device is discoverable by others
            ensureDiscoverable();
            return true;
        }
        return false;
    }

}