#ifndef __PINDEFS__
#define __PINDEFS__

/*
This is the definition of every pin that is put into our ardunio. it includes the 'addresses' that the servos 
are using, which are not pins in our ardunio but locally accessed addresses on the servo driver
*/

#define buttonPin 35
#define frontIRSensor 6
#define ledPinRed 47
#define ledPinGreen 45
#define ledPinBlue 43
#define rightFloorSensor 38
#define middleFloorSensor 36
#define leftFloorSensor 34

#define leftServoPin 9
#define rightServoPin 10
#define ladderServoPin 11
#define trigPin 22
#define echoPin 23

#endif