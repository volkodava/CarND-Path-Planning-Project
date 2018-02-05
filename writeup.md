# Project details

In current solution I use next parameters:
- Safe zone for the car to drive `49.5` is `30 m`.
- Safe zone for the car to make a turn is `30 m`.
- Car max speed is `49.5 mph`
- Car speed-up acceleration is `0.224 mph` which is `~5 metre/s^2` (`5 metre/s^2 = 0.1 m/sec (0.224mph) / 0.02sec`)
- Car slow down acceleration is `0.224 mph` which is `~5 metre/s^2`

To control the speed of the car and to avoid an accident we do calculate distance to the 
closest cars on the road. 

In scenario when car in front of us and is closer than safe distance and around us there is no space to change the lane line 
we apply slow down acceleration.

In scenario when car in front of us and is closer than safe distance and there is some safe (free) zone on either left or right lane line
we switch the lane line. Priority is given to the left lane line for the high speed traffic.
Scenarios handled with next routine:
```cpp
...
  if (too_close)
  {
    // lane for high speed preferable
    if (lane - 1 >= 0 && free_lanes[lane - 1])
    {
      lane -= 1;
    }
    else if (lane + 1 < free_lanes.size() && free_lanes[lane + 1])
    {
      lane += 1;
    }
    else
    {
      ref_vel -= CAR_SLOW_DOWN;
    }
  }
  else if (ref_vel < CAR_MAX_SPEED)
  {
    ref_vel += CAR_SPEED_UP;
  }
...
``` 

# Model Documentation

To build a smooth line between a waypoints we do use spline function. After we applied spline function 
we use generated points to feed back into simulator to generate trajectory for our car to follow. 
We know that every `20 ms` (`0.02`) the car moves to the next point on trajectory, 
so in order for the car to travel along the trajectory at desired velocity we do need to calculate number of points
using next equation:
```python
    double N = (target_dist / (.02 * ref_vel / 2.24));
```
To generate trajectory 2 previous points and 3 starter points (car at `30m`, `60m` and `90m`) with horizon of `30m`. 
See the code below:
```cpp
...
          // if previous size is almost empty, use the car as the starting reference
          if (prev_size < 2)
          {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else
          {
            // Redefine reference state as previous end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Franet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
...
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
...
```


# Results

Here's a [link to my project video result](./final_video.mov)
