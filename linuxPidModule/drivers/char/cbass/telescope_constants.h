//This file primarily defines local angle encoder (in 16 bit number) values to provide some direction and software limits for the antenna- these will be define differently on different antennas.

//of primary importance are the locations of the azimuth and elevation limit switches. Originally a single software limit was introduced to prevent command angles greater than certain values from being passed. This did not prevent the antenna momentum carrying the antenna into limits. 

//The definition of a 'Go slow zone' near the limits was introduced in an effort to counteract this momentum problem, while still allowing the antenna to operate near the limit zones. The actual max (min) speed in these zones is defined int the soft_lim() function. This was mainly introduced to avoid the need to throw motors into reverse with the possibility that this may damage the gears.

#define ALT_PID 0
#define AZ_PID 1

//the following two definitions should be adjusted if the angle encoder is replaced
#define AZIMUTH_ZERO 113000 //angle encoder position at due north (approximate)
#define ELEVATION_ZERO 32830 //angle encoder position at horizon (approximate) [should be calculated from the 'relatively well defined' stow lock position by subtracting 16384- it should be ensured that the encoder is physically positioned so that there is not possibility of an encoder overflow i.e 65535->0. so elevation zero >0 and elevation hi >16384 < 65535]

//NOTE WELL:approximate positions of limits at the hartrao setup (this will be different depending on the orientation)
//lo azimuth limit ~AZ Zero -75000
//hi azimuth limit ~ AZ_Zero + 24000
//HI elevation limit = lo_elevation limit /=16384 (i.e 90 degrees)


//NOTE THE FOLLOWING ARE CHANGED AFTER AN ENCODER IS SWOPPED OUT. THESE DEFINE SAFETY ANGLES USED IN THE USER SPACE SOFTWARE AS A SOFTWARE LIMIT CHECKER. PLEASE BE CAREFUL WITH THESE.

//1. these are for the kernel limit checking and also define the maximum limit values used in check_limit_values() which in turn provides a safety check for control.limits[i] used in the soft_lim() algorithm. This is the final software limit check. USE ELEVATION_LIMIT_HI AS THE POSITION AT STOW LOCK THEN ELEVATION_LIMIT_LO = ELEVATION_LIMIT_HI-16384 (090DEGREES)
#define AZIMUTH_LIMIT_LO 38000 
#define AZIMUTH_LIMIT_HI 137000
#define ELEVATION_LIMIT_LO 32830
#define ELEVATION_LIMIT_HI 49214
//2. these are for the user space limit checking- i.e a second order safety mechanism to slow the antenna velocity command (i.e pid output) to some predetermined speed when inside some encoder angle ranges.soft_lim()
#define AZIMUTH_SLOW_ZONE_HI 135000
#define AZIMUTH_SLOW_ZONE_LO 40000
#define ELEVATION_SLOW_ZONE_HI 48500
#define ELEVATION_SLOW_ZONE_LO 33500
//3. these define the initial maximum drivable encoder positions of the user space program (initialised in init_control_struct())).
#define AZIMUTH_SAFETY_LO 38000 
#define AZIMUTH_SAFETY_HI 135000
#define ELEVATION_SAFETY_LO 33300 
#define ELEVATION_SAFETY_HI 48500

//these define the maximum speeds in both directions allowed when in the safety zone defined by AZIMUTH_SLOW_ZONE_HI (see 2.) etc. These should only apply when the controller tried to drive 'deeper' into the zone. Driving out of the zone should not effected by the routine- 28 March this has not been implemented in the soft_lim() function yet. Should be trivial
#define AZ_SLOW_SPEED_POSITIVE 5000
#define AZ_SLOW_SPEED_NEGATIVE -5000
#define EL_SLOW_SPEED_POSITIVE 5000
#define EL_SLOW_SPEED_NEGATIVE -5000





#define MAX_TACHO_ALLOWABLE 10000
#define MIN_TACHO_ALLOWABLE -10000
//this section defines constants used when excepting commands from the network- it also defines ports that are used
#define LOCAL_SERVER_PORT 1501
#define REMOTE_SERVER_PORT 1500

//The following are not currently used
#define AZIMUTH_START 101000 //default start command position for the Azimuth axis
//these are for the user space limit checking
#define ELEVATION_START 40000 //default start command position for the ELEVATION axis
//these define the initial maximum drivable encoder positions of the user space program (initialised in init_control_struct())) These are used in soft_lim() routine and can be changed through a suitable TCP command 

#define MAX_AZ_SPEED 3000
#define MIN_AZ_SPEED -3000
#define MAX_ALT_SPEED 2000
#define MIN_ALT_SPEED -2000






