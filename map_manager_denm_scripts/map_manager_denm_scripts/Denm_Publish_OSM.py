#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from v2xvf_interfaces.msg import MapObject
from ros_etsi_its_msgs.msg import DENM
from sensor_msgs.msg import NavSatFix
import math
from math import *



M_PI = math.pi

RADIANS_PER_DEGREE = M_PI/180.0
DEGREES_PER_RADIAN = 180.0/M_PI

# Grid granularity for rounding UTM coordinates to generate MapXY.
grid_size = 100000.0;    # 100 km grid

# WGS84 Parameters
WGS84_A =6378137.0   # major axis
WGS84_B =6356752.31424518  # minor axis
WGS84_F =0.0033528107    # ellipsoid flattening
WGS84_E =0.0818191908    # first eccentricity
WGS84_EP =0.0820944379    # second eccentricity

# UTM Parameters
UTM_K0  =  0.9996               # scale factor
UTM_FE  = 500000.0             # false easting
UTM_FN_N = 0.0                  # false northing, northern hemisphere
UTM_FN_S = 10000000.0           # false northing, southern hemisphere
UTM_E2   = (WGS84_E*WGS84_E)    # e^2
UTM_E4   = (UTM_E2*UTM_E2)      # e^4
UTM_E6   = (UTM_E4*UTM_E2)      # e^6
UTM_EP2  = (UTM_E2/(1-UTM_E2))  # e'^2


def ll2xy(lat,lon,origin_lat,origin_lon):
    '''
    Geonav: Lat/Long to X/Y
    Convert latitude and longitude in dec. degress to x and y in meters
    relative to the given origin location.  Converts lat/lon and orgin to UTM and then takes the difference

    Args:
      lat (float): Latitude of location
      lon (float): Longitude of location
      orglat (float): Latitude of origin location
      orglon (float): Longitude of origin location

    Returns:
      tuple: (x,y) where...
        x is Easting in m (local grid)
        y is Northing in m  (local grid)
    '''

    outmy, outmx, outmzone = LLtoUTM(origin_lat,origin_lon)
    utmy, utmx, utmzone = LLtoUTM(lat,lon)
    if (not (outmzone==utmzone)):
        print('WARNING: geonav_conversion: origin and location are in different UTM zones!')
    y = utmy-outmy
    x = utmx-outmx
    return (x,y) 

def xy2ll(x, y, orglat, orglon):
    '''
    '''
    outmy, outmx, outmzone = LLtoUTM(orglat,orglon)
    utmy = outmy+y
    utmx = outmx+x
    return UTMtoLL(utmy,utmx,outmzone)

'''*
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 '''
def UTMLetterDesignator(Lat):
    
    LetterDesignator =""

    if 84 >= Lat >= 72:
        LetterDesignator = 'X'
    elif 72 > Lat >= 64:
        LetterDesignator = 'W'
    elif 64 > Lat >= 56:
        LetterDesignator = 'V'
    elif 56 > Lat >= 48:
        LetterDesignator = 'U'
    elif 48 > Lat >= 40:
        LetterDesignator = 'T'
    elif 40 > Lat >= 32:
        LetterDesignator = 'S'
    elif 32 > Lat >= 24:
        LetterDesignator = 'R'
    elif 24 > Lat >= 16:
        LetterDesignator = 'Q'
    elif 16 > Lat >= 8:
        LetterDesignator = 'P'
    elif 8 > Lat >= 0:
        LetterDesignator = 'N'
    elif 0 > Lat >= -8:
        LetterDesignator = 'M'
    elif -8 > Lat >= -16:
        LetterDesignator = 'L'
    elif -16 > Lat >= -24:
        LetterDesignator = 'K'
    elif -24 > Lat >= -32:
        LetterDesignator = 'J'
    elif -32 > Lat >= -40:
        LetterDesignator = 'H'
    elif -40 > Lat >= -48:
        LetterDesignator = 'G'
    elif -48 > Lat >= -56:
        LetterDesignator = 'F'
    elif -56 > Lat >= -64:
        LetterDesignator = 'E'
    elif -64 > Lat >= -72:
        LetterDesignator = 'D'
    elif -72 > Lat >= -80:
        LetterDesignator = 'C'
    else:
        LetterDesignator = 'Z' 
    return LetterDesignator


'''*
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 Retuns a tuple of (UTMNorthing, UTMEasting, UTMZone)
 '''
def LLtoUTM(Lat,Long):

  a = WGS84_A
  eccSquared = UTM_E2
  k0 = UTM_K0

  # Make sure the longitude is between -180.00 .. 179.9
  LongTemp = (Long+180.0)-int((Long+180.)/360.)*360.-180.

  LatRad = Lat*RADIANS_PER_DEGREE
  LongRad = LongTemp*RADIANS_PER_DEGREE
  ZoneNumber = int((LongTemp + 180.0)/6.0) + 1

  if ( Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
      ZoneNumber = 32
        # Special zones for Svalbard
  if ( Lat >= 72.0 and Lat < 84.0 ):
      if (      LongTemp >= 0.0  and LongTemp <  9.0 ): ZoneNumber = 31
      elif ( LongTemp >= 9.0  and LongTemp < 21.0 ): ZoneNumber = 33
      elif ( LongTemp >= 21.0 and LongTemp < 33.0 ): ZoneNumber = 35
      elif ( LongTemp >= 33.0 and LongTemp < 42.0 ): ZoneNumber = 37
  # +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1.0)*6.0 - 180.0 + 3.0
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE

  # Compute the UTM Zone from the latitude and longitude
  UTMZone = "%d%s"%(ZoneNumber,UTMLetterDesignator(Lat))
  #print("UTM Zone: %s"%(UTMZone))
  eccPrimeSquared = (eccSquared)/(1.0-eccSquared)
  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad))
  T = tan(LatRad)*tan(LatRad)
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad)
  A = cos(LatRad)*(LongRad-LongOriginRad)
  
  M = a*((1 - eccSquared/4.0 - 3.0*eccSquared*eccSquared/64.0
          - 5.0*eccSquared*eccSquared*eccSquared/256.0) * LatRad
         - (3.0*eccSquared/8.0 + 3.0*eccSquared*eccSquared/32.0
            + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(2.0*LatRad)
         + (15.0*eccSquared*eccSquared/256.0
            + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(4.0*LatRad)
         - (35.0*eccSquared*eccSquared*eccSquared/3072.0)*sin(6.0*LatRad))

  UTMEasting = (k0*N*(A+(1.0-T+C)*A*A*A/6.0
                      + (5.0-18.0*T+T*T+72*C
                         - 58.0*eccPrimeSquared)*A*A*A*A*A/120.0)
                + 500000.0)

  UTMNorthing = (k0*(M+N*tan(LatRad)
                     *(A*A/2.0+(5.0-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
                       + (61.0-58.0*T+T*T+600.0*C
                          - 330.0*eccPrimeSquared)*A*A*A*A*A*A/720.0)))
  if (Lat < 0):
      # 10000000 meter offset for southern hemisphere
      UTMNorthing += 10000000.0
  
  return (UTMNorthing, UTMEasting, UTMZone)

'''*
 * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 Returns (Lat, Lon, UTMZone)
 '''
def UTMtoLL(UTMNorthing,UTMEasting,UTMZone):
  k0 = UTM_K0
  a = WGS84_A
  eccSquared = UTM_E2
  e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared))

  x = UTMEasting - 500000.0;  # remove 500,000 meter offset for longitude
  y = UTMNorthing
  
  ZoneLetter = re.findall('([a-zA-Z])',UTMZone)[0]
  ZoneNumber = float( UTMZone.split(ZoneLetter)[0] )

  if (ZoneLetter <'N'):
      # remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0

  # +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6.0 - 180.0 + 3.0
  eccPrimeSquared = (eccSquared)/(1.0-eccSquared)
  M = y / k0
  mu = M/(a*(1.0-eccSquared/4.0-3.0*eccSquared*eccSquared/64.0
             -5.0*eccSquared*eccSquared*eccSquared/256.0))
  phi1Rad = mu + ((3.0*e1/2.0-27.0*e1*e1*e1/32.0)*sin(2.0*mu)
                  + (21.0*e1*e1/16.0-55.0*e1*e1*e1*e1/32.0)*sin(4.0*mu)
                  + (151.0*e1*e1*e1/96.0)*sin(6.0*mu))
  
  N1 = a/sqrt(1.0-eccSquared*sin(phi1Rad)*sin(phi1Rad))
  T1 = tan(phi1Rad)*tan(phi1Rad)
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad)
  R1 = a*(1.0-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5)
  D = x/(N1*k0)
  Lat = phi1Rad - ((N1*tan(phi1Rad)/R1)
                   *(D*D/2.0
                     -(5.0+3.0*T1+10.0*C1-4.0*C1*C1
                       -9.0*eccPrimeSquared)*D*D*D*D/24.0
                     +(61.0+90.0*T1+298.0*C1+45.0*T1*T1-252.0*eccPrimeSquared
                       -3.0*C1*C1)*D*D*D*D*D*D/720.0))
  
  Lat = Lat * DEGREES_PER_RADIAN
  
  Long = ((D-(1.0+2.0*T1+C1)*D*D*D/6.0
           +(5.0-2.0*C1+28.0*T1-3.0*C1*C1+8.0*eccPrimeSquared+24.0*T1*T1)
           *D*D*D*D*D/120.0)
          / cos(phi1Rad))
  Long = LongOrigin + Long * DEGREES_PER_RADIAN

  return (Lat, Long)

def location(data):
    global location_pos
    location_pos= NavSatFix()
    

    location_pos.header=data.header
    location_pos.altitude=data.altitude
    location_pos.latitude=data.latitude
    location_pos.longitude=data.longitude

class DenmPublisher(Node):

    def __init__(self):
        #calling constructor of the node class
        super().__init__('denm_node')
        self.denm_subscriber_= self.create_subscription(DENM, "/dummy_denm", self.denm_data, 10)
        self.denm_publisher_= self.create_publisher (MapObject, "/map_manager", 1000)
        self.get_logger().info("Dummy Node for DENM_Publish")
    
    def denm_data (self, ros_denm_msg : DENM):
        denm_msg = MapObject()

        #Accessing Denm Variables
        Header = ros_denm_msg.header
        Stamp = Header.stamp
        Frame_ID = Header.frame_id
        
        Its_Pdu_Header = ros_denm_msg.its_header
        Protocol_Version = Its_Pdu_Header.protocol_version
        Message_ID = Its_Pdu_Header.message_id
        Station_ID = Its_Pdu_Header.station_id

        Management_Container = ros_denm_msg.management
        Object_Lat = Management_Container.event_position.latitude
        Object_Lon = Management_Container.event_position.longitude


        Situation_Container = ros_denm_msg.situation
        Linked_Cause_Cause_Code = Situation_Container.linked_cause.cause_code
        Linked_Cause_Sub_Cause_Code = Situation_Container.linked_cause.sub_cause_code
        Event_Type_Cause_Code = Situation_Container.event_type.cause_code
        Event_Type_Sub_Cause_Code = Situation_Container.event_type.sub_cause_code

        Location_Container = ros_denm_msg.location


        #Converting Latitude and longitude only if they are not in lat and long
        #converted_obj_pose = xy2ll(x=Position_X, y=Position_Y, 
        #                        orglat=location_pos.latitude, orglon=location_pos.longitude)
        #print(converted_obj_pose)




        #Match GPS point to Mobileye reference point i.e convert from 48.1234567 to 481234567
        #THIPointLat = Object_Lat * 10000000   #Convert micro to degree
        #THIPointLon = Object_Lon * 10000000


        #Publishing the above data in Map Object format 
        denm_msg.position.latitude = Object_Lat
        denm_msg.position.longitude = Object_Lon
        denm_msg.type = "Level0"
        denm_msg.source = "Test1"
        denm_msg.source_id = ros_denm_msg.situation.linked_cause.cause_code
        denm_msg.id = "Test2"
        denm_msg.expiration_time = 1.3

        #Publishing on map
        self.denm_publisher_.publish(denm_msg)

def main(args=None):
    
    #initialise ros2 communication
    rclpy.init(args=args)

    #create a node
    denm_node = DenmPublisher()

    #spin the node
    rclpy.spin(denm_node)
    rclpy.shutdown()

#To run file directly from the terminal
if __name__ == '__main__':
    main()