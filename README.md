# Path-Planning

Path planning for an autonomous vehicle around a simulated track in presence of traffic.

## Goal

The goal of the project is to safely navigate around a virtual highway with other traffic that is driving at +-10 MPH of the 50 MPH speed limit. The car's (let's call it Ego) localization and sensor fusion data is available from the simulator. The car should avoid hitting other cars at all costs. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Available Data

The following data is available from the simulator:
Main car's localization Data (No Noise)

* The car's x position in map coordinates

* The car's y position in map coordinates

* The car's s position in frenet coordinates

* The car's d position in frenet coordinates

* The car's yaw angle in the map

* The car's speed in MPH

## Situation Awareness

The first task is to take stock of all the cars in the vicinity of Ego. In particular, the location and speed of 5 nearest cars are monitored. Four of these are in the left and right lane from Ego, two in front and two in the back. The fifth car is the nearest car in front in the same lane. In case the car is not traveling in the center lane, the position of the cars in either the left or the right lane will return a negative value so that it can be ignored in the subsequent steps.

Based on the location of the five cars, the possibility of collision in immediate future with either of those is calculated based on a time to collision parameter, which is tunable. The possibility of collision is calculated based on both the position and velocity of Ego and the other car involved. The time to collision parameter is actually converted into a distance to collision variable, which is checked against a tunable safe distance parameter to determine collision possibility. The distance to collision is much smaller for cars that are behind Ego compared to the cars that are in front of it. This is done to keep more distance in front to account for a braking event from the car in front.

The possibility of collision for each lane is passed to the next section to optimize the decision to pick the right lane to choose. It should be noted that while cars behind Ego in the neighboring lanes are monitored in case a lane change is necessary, the car behind Ego is not monitored.

## Check for Lane Availability

Before generating a path for the car to follow, it is necessary to check which lanes are available. If there is an imminent collision in the present lane, there are two possible actions that can be taken.
* If the left or the right lane is available, the preferred action is to change lane. If both lanes are available, the car picks the lane with more free path available in front.
* If lane change is not an option, the speed of the car is gradually reduced.

If there is no imminent collision in the present lane, the car continues in the lane. However, the car prefers to move to the center lane whenever there is enough free road available in front for all lanes. Staying in the center lane opens up more options to change lane in the future.

## Path Generation

When the destination lane for the car is decided, a few anchor points are generated for creating a spline. These anchor points are widely spaced and controls the length of the path to be generated. The spacing between the anchor points is critical. The car risks exceeding the lateral acceleration when the spacing is too low, because it allows for a sharp lane change. On the other hand, too much space between the anchor points will lead to a very smooth lane change but will take too long to complete a lane change, which is not desirable because other cars can take an action during this time that might lead to a potential collision.

A coordinate transformation to work in the car coordinate system is useful here so that only the forward distance from the car is sufficient to consider while generating future path points.

Future path points are generated using a spline and the length of the path to be generated is a tunable parameter. Normally a longer path is generated whenever there is a lane change involved to make sure the car arrives at the center of the lane before initiating another lane change should that become necessary.

The number of points to be generated is a function of the speed of the car to make sure the car has points available to follow for a fixed number of seconds. However, this will lead to very few points generated when the car is moving very slow. Therefore, a minimum number of points is always generated irrespective of the car's speed.

Once the future points are generated, they are transformed back to the global coordinate system for the simulator to consume.

## Path Following

The general strategy is to keep a list of points for the future path that Ego will follow. However, if Ego does not consume all the points available in one iteration, the remaining points are not thrown away. Instead, the newly generated path points are augmented to the existing points. Not only it limits the amount of calculation that needs to be performed at each iteration, of also guarantees that the car follows a smooth trajectory.

## Speed Control

Before the future points are passed back to the simulator, the speed of the car is checked. The speed is increased if it is moving at a speed below the speed limit and there is no imminent collision. The speed is decreased if it is approaching the speed limit or there is a car within striking distance in front and lane change is not an option. The amount of increment, either positive or negative, is set to make sure the car does not exceed the allowed limit for tangential acceleration.

## Possible Improvements

Although Ego moves reasonably well in the traffic, there is one situation where it drives inefficiently. Ego can be easily "boxed in" - when it is forced to drive in one of the side lanes at a speed much below the speed limit. It happens when there is a slow moving car in front of Ego and another car in the center lane to prevent it from changing lane even though the third lane may be empty. To come out of the situation, Ego needs to first reduce its speed and initiate a lane change to go from one side lane to another.
