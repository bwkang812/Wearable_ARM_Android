package com.example.android.BluetoothChat;

import android.content.Context;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.LocationProvider;
import android.os.Bundle;

/**
 * Created by Kim on 2016-03-27.
 */
public class LocationClass
{
    // getSystemService가 Context를 필요로 하기 때문에 메인 Activity에서 Context를 가져옴
    private Context mContext;

    // 위치정보를 위해서 LocationManager선언, 위치제공자 저장을 위한 mProvider
    private LocationManager mLocation;
    private String mProvider;

    // OnLocationListener를 사용하기 위해서 mOnLocationListener
    private OnLocationListener mOnLocationListener = null;

    // Context를 받기 위한 생성자
    public LocationClass(Context context)
    {
        mContext = context;
    }

    // 위치 정보를 받기 위해서 위치제공자 목록을 조사하고 최적의 위치제공자 검색
    public void init_Location()
    {
        // 위치제공자 목록 조사
        mLocation = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
        // 최적의 위치제공자 검색
        mProvider = mLocation.getBestProvider(new Criteria(), true);
    }

    // 위치정보를 갱신하기 위한 리스너 등록
    public void onResumeLocation()
    {
        // 최적의 위치제공자를 1분이 지날때마다 리스너를 호출하는데
        // 100미터 이상의 거리 차이가 있을경우 리스너를 호출한다.
        mLocation.requestLocationUpdates(mProvider, 6000, 100, mListener);
    }

    // 위치정보 갱신을 멈춤(더 이상 정보를 받지 않겠다)
    public void onPauseLocation()
    {
        // 리스너 받는것을 해제
        mLocation.removeUpdates(mListener);
    }

    // 리스너를 통해서 Activity에 데이터를 보내기 위한 함수
    public interface OnLocationListener
    {
        void onRecvMessage(String sData);
    }


    // 리스너를 통해서 Activity에 데이터를 보내기 위한 변수
    public class LParam
    {
        public String resultMsg;
    }

    //OnLocationListener를 mOnLocationListener에 등록
    public void setOnLocationListener(OnLocationListener listener)
    {
        mOnLocationListener = listener;
    }

    //위치 정보를 감지하는 리스너
    LocationListener mListener = new LocationListener()
    {
        // 위치 정보가 변경 되었을 때 호출
        public void onLocationChanged(Location location)
        {
            String sloc = String.format("위도:%.4f\n경도:%.4f",
                    location.getLatitude(), location.getLongitude());
            mOnLocationListener.onRecvMessage(sloc);
        }

        // 위치제공자가 변경 되었을 때 호출
        public void onProviderDisabled(String provider)
        {

        }

        // 위치제공자가 활성화 되었을 때 호출
        public void onProviderEnabled(String provider)
        {

        }

        // 위치제공자의 상태가 변경 되었을 때 호출
        public void onStatusChanged(String provider, int status, Bundle extras)
        {
            String sStatus = "";
            switch (status) {
                case LocationProvider.OUT_OF_SERVICE:
                    sStatus = "범위 벗어남";
                    break;
                case LocationProvider.TEMPORARILY_UNAVAILABLE:
                    sStatus = "일시적 불능";
                    break;
                case LocationProvider.AVAILABLE:
                    sStatus = "사용 가능";
                    break;
            }
            // 메인 Activity에 sStatus를 보냄
            mOnLocationListener.onRecvMessage(sStatus);
        }
    };

}