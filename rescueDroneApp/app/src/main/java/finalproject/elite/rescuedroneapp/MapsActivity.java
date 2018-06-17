package finalproject.elite.rescuedroneapp;

import android.Manifest;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.firebase.database.ChildEventListener;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

import java.util.ArrayList;
import java.util.Map;

public class MapsActivity extends AppCompatActivity implements OnMapReadyCallback,
        GoogleApiClient.ConnectionCallbacks,
        GoogleApiClient.OnConnectionFailedListener,
        LocationListener,
        View.OnClickListener {

    private static final String TAG = "MapsActivity";

    private GoogleMap mMap;
    private GoogleApiClient mGoogleApiClient;
    private LocationRequest mLocationRequest;
    private Location mLastLocation;
    //private Location destLocation;
    private LatLng destLatLng;
    private ArrayList<LatLng> destPoints  = new ArrayList<>();;
    private Marker markerToAdd;
    private ArrayList<Marker> destMarkers = new ArrayList<>();;
    private ArrayList<Polyline> allPolyLines = new ArrayList<>();
    private FirebaseDatabase mFirebaseDatabase;
    private DatabaseReference mDatabaseReference;
    private LatLng currLatLng;

    public static final int MY_PERMISSIONS_REQUEST_LOCATION = 99;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);

        if (android.os.Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            checkLocationPermission();
        }
        // Initializing
        //destMarkers = new ArrayList<>();
        //allPolylines = new ArrayList<>();
        //destPoints = new ArrayList<>();
        //destLocation = new Location("");

        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        mFirebaseDatabase = FirebaseDatabase.getInstance();
        mDatabaseReference = mFirebaseDatabase.getReference();


        //database reference pointing to demo node
        DatabaseReference demoRef = mDatabaseReference.child("locations");
        demoRef.child("location0").getDatabase();
    }


    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */


    @Override
    public void onMapReady(final GoogleMap googleMap) {
        Log.d(TAG, "on map readyyyyyyyyyyy");
        mMap = googleMap;
        mMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (ContextCompat.checkSelfPermission(this,
                    Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
                buildGoogleApiClient();
                mMap.setMyLocationEnabled(true);
            }
        } else {
            buildGoogleApiClient();
            mMap.setMyLocationEnabled(true);

        }

        // Read from the database

    }



    private void getChild(DataSnapshot dataSnapshot) {
        mMap.clear();
        Log.d(TAG, String.valueOf(dataSnapshot));
            for (DataSnapshot ds : dataSnapshot.getChildren()) {
                Map<String, String> map = (Map<String, String>) ds.getValue();
                Log.d(TAG, String.valueOf(map));
                String sLatitude = String.valueOf(map.get("latitude"));
                String sLongitude = String.valueOf(map.get("longitude"));
                double dLatitude = Double.valueOf(sLatitude);
                double dLongitude = Double.valueOf(sLongitude);
                destLatLng = new LatLng(dLatitude, dLongitude);

                LatLng origin = new LatLng(mLastLocation.getLatitude(), mLastLocation.getLongitude());

                destPoints.add(destLatLng);
                Log.d(TAG, " dest points in child: " + destPoints);
                Log.d(TAG, "in get child!!!!!!!!!!!!!!!!!!");
                for (LatLng point : destPoints) {
                    addPointMarker(point);
                    drawLine(origin, point);
                    mMap.moveCamera(CameraUpdateFactory.newLatLng(origin));

                }
            }
    }

    @Override
    public void onClick(View view) {

    }

    protected synchronized void buildGoogleApiClient() {
        //used for configure client:
        mGoogleApiClient = new GoogleApiClient.Builder(this)
                .addConnectionCallbacks(this) //when client connected or disconnected.
                .addOnConnectionFailedListener(this) //covers scenarios of failed attempt of connect client to service.
                .addApi(LocationServices.API) //adds the LocationServices API endpoint from Google Play Services.
                .build();
        mGoogleApiClient.connect(); //A client must be connected before executing any operation.
    }

    @Override
    public void onLocationChanged(Location location) {

        Log.d(TAG, "in location changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Log.d(TAG, "destpoints in location changed: " + destPoints);
        Log.d(TAG, "destMarker in location changed: " + destMarkers);
        Log.d(TAG, "poly lines in location changed: " + allPolyLines);
        mLastLocation = location;

        //place current location marker:
        LatLng latLng = new LatLng(location.getLatitude(), location.getLongitude()); //getting coordinates of current location.

        //Move map camera:
        //mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(latLng, 16f));


        currLatLng = latLng;

        for (Polyline polyline: allPolyLines) {
            polyline.remove();
        }
        redrawLine();
        changeDistanceOnMarker();
    }

    @Override
    protected void onPause() {
        super.onPause();
        //Stop location updates:
        if (mGoogleApiClient != null) {
            LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, this);

        }
    }

    private void redrawLine() {
        Log.d(TAG, "in re draw!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

        for (int i = 0; i < destPoints.size(); i++) {
            Log.d(TAG, "in for in re draw!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            LatLng point = destPoints.get(i);
            drawLine(currLatLng, point);
        }
    }

    private void drawLine(LatLng origin, LatLng destination) {
        Log.d(TAG,"add line!!!!!!!!!!!!!!!!!!!!!!!");
        Polyline polyline = mMap.addPolyline(new PolylineOptions()
                .clickable(true).add(origin, destination).color(getResources().
                        getColor(R.color.colorBlue)).width(10));
        allPolyLines.add(polyline);
    }

    private void changeDistanceOnMarker() {
        Log.d(TAG,"change distance!!!!!!!!!!!!!!!!!!!!!!!");

        for (Marker marker: destMarkers) {
            LatLng latLng =  marker.getPosition();

            marker.setTitle(String.valueOf(calculateDistance(latLng) + "m"));
            if(marker.isInfoWindowShown()) {
                marker.showInfoWindow();
            }

        }
    }

    private void addPointMarker(LatLng point) {
        Log.d(TAG,"add Marker!!!!!!!!!!!!!!!!!!!!!!!");

        markerToAdd = mMap.addMarker(new MarkerOptions().position(point)
                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED))
                .title(String.valueOf(calculateDistance(point))+ "m"));
        destMarkers.add(markerToAdd);
    }

    private double calculateDistance(LatLng latLng) {
        Location destLocation = new Location("");
        destLocation.setLatitude(latLng.latitude);
        destLocation.setLongitude(latLng.longitude);
        return (int)mLastLocation.distanceTo(destLocation);
    }


    @Override
    public void onConnected(@Nullable Bundle bundle) {
        Log.d(TAG, "connecteddddddddddddddddddd");
        Log.d(TAG, "dddd" + destPoints.toString());
        mLocationRequest = new LocationRequest();
        mLocationRequest.setInterval(1000);
        mLocationRequest.setFastestInterval(1000);
        mLocationRequest.setPriority(LocationRequest.PRIORITY_BALANCED_POWER_ACCURACY);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            LocationServices.FusedLocationApi.requestLocationUpdates
                    (mGoogleApiClient, mLocationRequest, this);

        }

        //destPoints.clear();

        // Read from the database
        mDatabaseReference.addChildEventListener(new ChildEventListener() {
            @Override
            public void onChildAdded(DataSnapshot dataSnapshot, String s) {
                if (mLastLocation != null) {
                    getChild(dataSnapshot);
                }
            }

            @Override
            public void onChildChanged(DataSnapshot dataSnapshot, String s) {
                //destPoints.clear();
                mMap.clear();
                if (mLastLocation != null) {
                    getChild(dataSnapshot);
                }
            }

            @Override
            public void onChildRemoved(DataSnapshot dataSnapshot) {
                // when there is no locations at all (rescue-drone data have no childes):
                destPoints.clear();
                mMap.clear();

            }

            @Override
            public void onChildMoved(DataSnapshot dataSnapshot, String s) {
            }

            @Override
            public void onCancelled(DatabaseError error) {
                // Failed to read value
                Log.w(TAG, "Failed to read value.", error.toException());
            }


        });


    }

    @Override
    protected void onResume() {
        super.onResume();

        if (mGoogleApiClient != null) {
            mGoogleApiClient.connect();
        }
        Log.d(TAG, "resumeeeeeeeeeeeeeeeeeee");
    }

    @Override
    public void onConnectionSuspended(int i) {

    }

    @Override
    public void onConnectionFailed(@NonNull ConnectionResult connectionResult) {

    }

    /**
     * app will ask the user for permission and
     * proceed with the operation according to the following permission given,
     *
     * @return: permission granted or permission denied
     */
    public boolean checkLocationPermission() {
        if (ContextCompat.checkSelfPermission(this,
                Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {

            // Asking user if explanation is needed -
            // return true if the app has requested this permission previously
            // and the user denied the request.
            // It will return false if user has chosen Don’t ask again option
            // when it previously asked for permission.
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                    Manifest.permission.ACCESS_FINE_LOCATION)) {

                // Show an expanation to the user *asynchronously* -- don't block
                // this thread waiting for the user's response! After the user
                // sees the explanation, try again to request the permission.

                //Prompt the user once explanation has been shown
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                        MY_PERMISSIONS_REQUEST_LOCATION);


            } else {
                // No explanation needed, we can request the permission.
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                        MY_PERMISSIONS_REQUEST_LOCATION);
            }
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_LOCATION: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // Permission was granted.
                    if (ContextCompat.checkSelfPermission(this,
                            Manifest.permission.ACCESS_FINE_LOCATION)
                            == PackageManager.PERMISSION_GRANTED) {

                        if (mGoogleApiClient == null) {
                            buildGoogleApiClient();
                        }
                        mMap.setMyLocationEnabled(true);
                    }

                } else {

                    // Permission denied, Disable the functionality that depends on this permission.
                    Toast.makeText(this, "permission denied", Toast.LENGTH_LONG).show();
                }
                return;
            }

            // other 'case' lines to check for other permissions this app might request.
            //You can add here other case statements according to your requirement.
        }
    }
}
