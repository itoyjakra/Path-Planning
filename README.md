# Path-Planning
Path planning for autonomous vehicle

## Situation Awareness

The first task is to take stock of all the cars in the vicinity. In particular, the location and speed of 5 cars are monitored. Four of these are in the left and right lane from the ego car, two in front and two in the back. The fifth car is the nearest car in front in the same lane. In case the car is not traveling in the center lane, the position of the cars in either the left or the right lane will return a negative value so that it can be ignored in the subsequent steps.

Based on the location of the five cars, the possibility of collision in immediate future with either of those is calculated based on a time to collision parameter, which is tunable. The possibility of collision is calculated based on both the position and velocity of ego and the other car involved. The time to collision parameter is actually converted into a distance to collision variable, which is checked against a tunable safe distance parameter to determine collision possibility. The distance to collision is much smaller for cars that are behind ego compared to the cars that are in front of it. This is done to keep more distance in front to account for a braking event from the car in front.

The possibility of collision for each lane is passed to the next section to optimize the decision to pick the right lane to choose. It should be noted that while cars behind the ego car in the neighboring lane are monitored in case a lane change is necessary, the car behind ego is not monitored.

## Check for Lane Availability

Before generating a path for the car to follow, it is necessary to check which lanes are available. If there is an imminent collision in the present lane, there are two possible actions that can be taken.
* If the left or the right lane is available, the preferred action is to change lane. If both lanes are available, the car picks the lane with more free path available in front.
* If lane change is not an option, the speed of the car is gradually reduced.

If there is no imminent collision in the present lane, the car continues in the lane. However, the car prefers to move to the center lane whenever there is enough free road available in front for all lanes. Staying in the center lane opens up more options to change lane in the future.

## Path Generation

When the destination lane for the car is decided, a few anchor points are generated for creating a spline. These anchor points are widely spaced and controls the length of the path to be generated.

A coordinate transformation to work in the car coordinate system is useful here so that only the forward distance from the car is sufficient to consider while generating future path points.

Future path points are generated using the spline and the length is the path to be generated is a tunable parameter. Normally a longer path is generated when there is a lane change involved to make sure the car arrives at the center of the lane before initiating another lane change should that become necessary.

The number of points to be generated is a function of the speed of the car to make sure the car has points available to follow for a fixed number of seconds. However, this will lead to very few points generated when the car is moving very slow. Therefore, a minimum number of points is always generated.

Once the future points are generated, they are transformed back to the global coordinate system for the simulator to consume.

## Speed Control

Before the future points are passed back to the simulator, the speed of the car is checked. The speed is increased if it is moving at a speed below the speed limit and there is no imminent collision. The speed is decreased if it is approaching the speed limit or there is a car within striking distance in front and lane change is not an option.
